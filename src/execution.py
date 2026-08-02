"""Where the solvers actually run, and how many cores they are given.

Both CFD backends already ran their solver as an external process in a Linux
environment reached through a shim; WSL was simply the only shim. This module
makes the shim a choice:

``native``
    The solver is on ``PATH``. No translation, no virtualisation layer.
``docker``
    The solver lives in a pinned image. The case directory is bind-mounted at
    ``/case``, so path translation is a constant rather than a ``wslpath``
    subprocess per call.
``wsl``
    The solver is installed inside a WSL distro. The original path, kept
    because it is what an existing install looks like.

Auto-detection prefers ``native`` (fastest), then ``docker`` (reproducible:
it pins OpenFOAM 13 and SU2 8.4.0 instead of inheriting whatever the machine
happens to have), then ``wsl``. Set ``AERO_EXECUTION`` to force one.

The images are CPU-only on purpose. GPU acceleration in OpenFOAM lives on the
ESI fork rather than the Foundation build this tool drives, only touches the
linear solve, and wants meshes an order of magnitude larger than the ones we
generate; the meshers have no GPU path at all. Cores are the lever here, which
is what ``default_processes`` is for.
"""

from __future__ import annotations

import contextlib
import os
import shlex
import shutil
import subprocess
import threading
import uuid
from dataclasses import dataclass, field
from pathlib import Path


# Fraction of the visible cores a solve is given when nothing says otherwise.
# Not all of them: the mesher, the GUI and the operating system still need a
# core, and an oversubscribed MPI run is slower than a correctly sized one.
DEFAULT_CORE_FRACTION = 0.8

# Pinned images. Override per-machine without touching the source.
OPENFOAM_IMAGE = os.environ.get("AERO_OPENFOAM_IMAGE", "aero-drag-tool/openfoam:13")
SU2_IMAGE = os.environ.get("AERO_SU2_IMAGE", "aero-drag-tool/su2:8.4.0")

# Where the case directory is mounted inside a container.
CONTAINER_CASE_DIR = "/case"

MODES = ("native", "docker", "wsl")


# --------------------------------------------------------------------------
# Stopping a solve that is already running
# --------------------------------------------------------------------------


class Cancelled(BaseException):
    """The solve on this thread was asked to stop.

    Deliberately a :class:`BaseException`. The solver stack turns any ``Exception``
    into a failed run with a log excerpt -- correct for a solver that crashed,
    wrong for a user who pressed stop. Inheriting from ``BaseException`` puts
    this in the same class as ``KeyboardInterrupt``: it unwinds through those
    handlers instead of being recorded as a failure.
    """


@dataclass
class CancelScope:
    """The kill switch for one solve, and what it currently has running.

    A solve is a long chain of short-lived external processes (mesh, decompose,
    solve, reconstruct), so stopping it is two things: kill whatever is running
    right now, and refuse to start the next one. Both live here.
    """

    cancelled: bool = False
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _processes: set = field(default_factory=set, repr=False)
    _containers: set = field(default_factory=set, repr=False)

    def cancel(self) -> None:
        """Ask the solve to stop, killing anything already in flight."""
        with self._lock:
            self.cancelled = True
            processes = list(self._processes)
            containers = list(self._containers)

        for name in containers:
            # A detached client leaves the container running, so the container
            # has to be stopped by name rather than by killing `docker run`.
            with contextlib.suppress(OSError, subprocess.SubprocessError):
                subprocess.run(
                    ["docker", "kill", name],
                    capture_output=True,
                    text=True,
                    timeout=30,
                )
        for process in processes:
            with contextlib.suppress(OSError):
                process.kill()

    def raise_if_cancelled(self) -> None:
        if self.cancelled:
            raise Cancelled("The solve was stopped")

    def _attach(self, process, container: str = "") -> None:
        with self._lock:
            self._processes.add(process)
            if container:
                self._containers.add(container)

    def _detach(self, process, container: str = "") -> None:
        with self._lock:
            self._processes.discard(process)
            if container:
                self._containers.discard(container)


# Keyed by thread: the queue runs one solve at a time on its own thread, and
# every solver process is spawned from that same thread. That makes the scope
# discoverable from deep inside a backend without threading a token through
# every call between here and the solver.
_scopes: dict[int, CancelScope] = {}
_scopes_lock = threading.Lock()


def current_scope() -> CancelScope | None:
    with _scopes_lock:
        return _scopes.get(threading.get_ident())


@contextlib.contextmanager
def cancel_scope():
    """Make everything this thread runs from here on stoppable."""
    scope = CancelScope()
    ident = threading.get_ident()
    with _scopes_lock:
        _scopes[ident] = scope
    try:
        yield scope
    finally:
        with _scopes_lock:
            _scopes.pop(ident, None)


def checkpoint() -> None:
    """Give up here if a stop was asked for. Cheap; call between solver steps."""
    scope = current_scope()
    if scope is not None:
        scope.raise_if_cancelled()


def run_process(
    command: list[str],
    *,
    check: bool = False,
    capture_output: bool = False,
    timeout: int | None = None,
    cwd: str | Path | None = None,
    stdout=None,
    stderr=None,
    container: str = "",
) -> subprocess.CompletedProcess[str]:
    """``subprocess.run``, but killable while it runs.

    Registered with the calling thread's :class:`CancelScope` so a stop request
    from the web thread can reach the solver. Outside a scope this is exactly
    ``subprocess.run``.
    """
    scope = current_scope()
    if scope is not None:
        scope.raise_if_cancelled()

    if capture_output:
        stdout, stderr = subprocess.PIPE, subprocess.PIPE

    process = subprocess.Popen(  # noqa: S603 - commands are built here, not user input
        command,
        cwd=str(cwd) if cwd is not None else None,
        stdout=stdout,
        stderr=stderr,
        text=True,
    )
    if scope is not None:
        scope._attach(process, container)
    try:
        out, err = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.kill()
        process.communicate()
        raise
    finally:
        if scope is not None:
            scope._detach(process, container)

    # A killed solver exits non-zero and writes a truncated log. Ask first, so
    # that reads as "stopped" rather than as a solver that fell over.
    if scope is not None:
        scope.raise_if_cancelled()

    completed = subprocess.CompletedProcess(command, process.returncode, out, err)
    if check:
        completed.check_returncode()
    return completed


# --------------------------------------------------------------------------
# How many cores
# --------------------------------------------------------------------------


def _cgroup_cpu_limit() -> int | None:
    """CPU limit imposed by a cgroup, or None if unlimited/not applicable.

    Matters when the *tool itself* runs in a container: ``os.cpu_count`` reports
    the host's cores there, so an 80% default computed from it would oversubscribe
    the container by a wide margin.
    """
    v2 = Path("/sys/fs/cgroup/cpu.max")
    try:
        if v2.exists():
            parts = v2.read_text().split()
            if len(parts) == 2 and parts[0] != "max":
                quota, period = int(parts[0]), int(parts[1])
                if quota > 0 and period > 0:
                    return max(1, quota // period)
        quota_path = Path("/sys/fs/cgroup/cpu/cpu.cfs_quota_us")
        period_path = Path("/sys/fs/cgroup/cpu/cpu.cfs_period_us")
        if quota_path.exists() and period_path.exists():
            quota = int(quota_path.read_text().strip())
            period = int(period_path.read_text().strip())
            if quota > 0 and period > 0:
                return max(1, quota // period)
    except (OSError, ValueError):
        return None
    return None


def available_cores() -> int:
    """Cores this process can actually use, not cores the machine has.

    CPU affinity and cgroup quotas both narrow the former without touching the
    latter, and both are normal in the environments this tool runs in.
    """
    if hasattr(os, "sched_getaffinity"):
        cores = len(os.sched_getaffinity(0))  # type: ignore[attr-defined]
    else:
        cores = os.cpu_count() or 1
    limit = _cgroup_cpu_limit()
    if limit is not None:
        cores = min(cores, limit)
    return max(1, cores)


def default_processes() -> int:
    """80% of the visible cores, at least one."""
    override = os.environ.get("AERO_PROCESSES")
    if override:
        try:
            return max(1, int(override))
        except ValueError:
            pass
    return max(1, int(available_cores() * DEFAULT_CORE_FRACTION))


def resolve_processes(requested: int | None) -> int:
    """Turn a possibly-absent request into a usable process count.

    ``None`` means "decide for me" and yields :func:`default_processes`. An
    explicit request is honoured but clamped to the cores that exist, because
    an MPI run with more ranks than cores thrashes rather than scales.
    """
    if requested is None:
        return default_processes()
    try:
        count = int(requested)
    except (TypeError, ValueError):
        return default_processes()
    return max(1, min(count, available_cores()))


# --------------------------------------------------------------------------
# Docker
# --------------------------------------------------------------------------


def docker_available() -> bool:
    """A docker CLI that can actually reach a daemon."""
    if shutil.which("docker") is None:
        return False
    try:
        probe = subprocess.run(
            ["docker", "info", "--format", "{{.ServerVersion}}"],
            capture_output=True,
            text=True,
            timeout=30,
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return probe.returncode == 0


def image_present(image: str) -> bool:
    try:
        probe = subprocess.run(
            ["docker", "image", "inspect", image],
            capture_output=True,
            text=True,
            timeout=30,
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return probe.returncode == 0


# --------------------------------------------------------------------------
# Running a command wherever the solver lives
# --------------------------------------------------------------------------


@dataclass
class Runner:
    """A place to run solver commands, and how many ranks to give them."""

    mode: str
    processes: int = 1
    image: str = ""
    distro: str = ""
    # Sourced or exported before every command, so the solver is on PATH.
    preamble: str = ""
    # Path to the solver binary, where the backend needs to name it explicitly.
    executable: str = ""
    # WSL installs from setup.sh live under root; containers already are root.
    as_root: bool = False
    label: str = ""

    def describe(self) -> str:
        where = {
            "native": "PATH",
            "docker": f"Docker {self.image}",
            "wsl": f"WSL {self.distro}",
        }.get(self.mode, self.mode)
        parallel = f", {self.processes} process{'es' if self.processes != 1 else ''}"
        name = self.label or self.executable or "solver"
        return f"{name} via {where}{parallel}"

    def case_path(self, case_dir: str | Path) -> str:
        """The case directory as the *solver* sees it."""
        if self.mode == "docker":
            return CONTAINER_CASE_DIR
        if self.mode == "wsl":
            return _wsl_path(self.distro, case_dir, as_root=self.as_root)
        return str(Path(case_dir).resolve())

    def bash(
        self,
        script: str,
        case_dir: str | Path | None = None,
        check: bool = True,
        capture_output: bool = True,
        timeout: int | None = None,
    ) -> subprocess.CompletedProcess[str]:
        """Run ``script`` under bash where the solver lives.

        ``case_dir`` is a host path; it is mounted or translated as the mode
        requires and becomes the working directory.
        """
        inner = self.preamble
        if case_dir is not None and self.mode != "docker":
            # Docker sets the working directory on the command line instead,
            # so the container never has to know the host's path layout.
            cd = f"cd {shlex.quote(self.case_path(case_dir))}"
            inner = f"{inner} && {cd}" if inner else cd
        inner = f"{inner} && {script}" if inner else script

        container = ""
        if self.mode == "docker":
            # Named so that stopping the run can reach the container itself;
            # killing the `docker run` client would leave it solving.
            container = f"aero-{uuid.uuid4().hex[:12]}"
            command = _docker_command(self.image, case_dir, inner, name=container)
        elif self.mode == "wsl":
            command = _wsl_command(self.distro, inner, as_root=self.as_root)
        else:
            command = ["bash", "-lc", inner]

        return run_process(
            command,
            check=check,
            capture_output=capture_output,
            timeout=timeout,
            container=container,
        )


def _docker_command(
    image: str, case_dir: str | Path | None, script: str, name: str = ""
) -> list[str]:
    command = ["docker", "run", "--rm"]
    if name:
        command += ["--name", name]
    if case_dir is not None:
        source = Path(case_dir).resolve()
        # --mount rather than -v: its source=,target= syntax is not confused by
        # the colon in a Windows drive letter.
        command += [
            "--mount",
            f"type=bind,source={source.as_posix()},target={CONTAINER_CASE_DIR}",
            "-w",
            CONTAINER_CASE_DIR,
        ]
    if hasattr(os, "getuid"):
        # Otherwise every file the solver writes lands on the host owned by
        # root. Windows hosts do not have the problem and do not have getuid.
        command += ["-u", f"{os.getuid()}:{os.getgid()}", "-e", "HOME=/tmp"]  # type: ignore[attr-defined]
    command += [image, "bash", "-lc", script]
    return command


def _wsl_command(distro: str, script: str, as_root: bool = False) -> list[str]:
    command = ["wsl", "-d", distro]
    if as_root:
        command += ["-u", "root"]
    return command + ["-e", "bash", "-lc", script]


def _wsl_path(distro: str, path: str | Path, as_root: bool = False) -> str:
    completed = subprocess.run(
        _wsl_command(distro, f"wslpath -a {shlex.quote(str(Path(path)))}", as_root=as_root),
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip()


def forced_mode() -> str | None:
    """An execution mode pinned by the environment, if any."""
    mode = os.environ.get("AERO_EXECUTION", "").strip().lower()
    if mode in MODES:
        return mode
    return None
