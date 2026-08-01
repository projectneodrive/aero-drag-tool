"""Entry point for the SU2 track.

This used to be a standalone script that could only mesh a hard-coded 1 m
cube. The case generation now lives in :mod:`su2` and works on any STL, with
a wind vector, hull attitude and an optional road; this file just forwards to
the shared CLI with SU2 selected.

    python drag.py info
    python drag.py new --stl hull.stl --ground 0.15 -o case.aero.json
    python drag.py run case.aero.json
"""

from __future__ import annotations

import sys

from runner import main


if __name__ == "__main__":
    argv = sys.argv[1:]
    # Default this entry point to the SU2 backend unless told otherwise.
    if argv and argv[0] in {"run", "export", "new"} and "--solver" not in argv:
        argv = argv + ["--solver", "su2"]
    raise SystemExit(main(argv))
