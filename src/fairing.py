"""Automatic fairing generation from an arbitrary payload STL.

The question "should this payload be one fairing or several?" is not a discrete
choice to make up front -- it is a *length scale*. Two lumps belong to the same
body when the gap between them is small compared to the skin you would wrap
around them, and belong to separate bodies otherwise.

So instead of deciding, we sweep the scale. A morphological closing (dilate by
r, then erode by r) hugs each lump at small r and bridges the gaps at large r,
and the topology changes on its own. Plotting the number of connected
components against r gives a staircase; the **plateaus are the candidate
designs**, and a wide plateau means a topology that is robust rather than an
accident of two things nearly touching.

Two refinements matter:

* The closing is **anisotropic**, elongated along the flow. Two lumps in line
  should merge much more readily than two lumps side by side: bridging in-line
  lumps costs almost no frontal area and removes a wake-impingement problem,
  while bridging side-by-side lumps means filling the whole span between them.
* The surface comes from an isosurface of a **signed distance field** at a
  positive level, not from the raw voxel mask. That gives a smooth, sub-voxel
  skin and makes the clearance gap exact by construction, so the payload is
  guaranteed to fit with room to spare.

The closing decides *topology*, not *profile*. It is bounded above by the
convex hull, so on a convex payload it is a no-op and the skin would come out
as the payload plus clearance -- a rounded cube for a cube, which is a terrible
thing to fly. The profile comes from a second stage, ``streamline_mask``: the
minimal envelope whose cross-sections grow no steeper than a nose angle and
shrink no steeper than a tail angle along the flow. That is not a shape search;
it encodes the two rules that dominate bluff-body drag -- keep the tail shallow
enough to stay attached, and never carry more section than the payload forces
-- and computes the smallest body satisfying them. A cube in, a teardrop out,
with the tail length set by the taper limit rather than by taste. Whether a
given taper limit was worth its wetted area is then the solvers' question, not
this module's.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import trimesh
from scipy import ndimage

from metrics import frontal_area, normalize


# Grid resolution along the payload's longest axis. 128 keeps a 2 m vehicle at
# roughly 15 mm voxels, which resolves every gap that matters for packaging.
DEFAULT_RESOLUTION = 128
MAX_VOXELS = 40_000_000

# Streamwise elongation of the structuring element. Around 3 makes an in-line
# gap close at a third of the radius a spanwise gap of the same size needs.
DEFAULT_ANISOTROPY = 3.0

# Configurations with more separate bodies than this are not worth proposing.
MAX_SENSIBLE_BODIES = 8

# Gap between two bodies, as a fraction of the body scale, below which the
# channel between them chokes and the configuration should be merged instead.
CHOKED_GAP_RATIO = 0.06

# Taper limits for the streamlined envelope, in degrees from the flow axis.
# The tail is the one that matters: much past ~15 deg the boundary layer on the
# afterbody separates and the wake swallows whatever the shorter tail saved.
# The nose is far more forgiving -- the flow there is accelerating -- so it can
# be blunt without much penalty, and 45 deg keeps the nose usefully short.
DEFAULT_NOSE_ANGLE_DEG = 45.0
DEFAULT_TAIL_ANGLE_DEG = 12.0

# Outside these the envelope stops meaning anything: a tail shallower than
# 5 deg is many body-lengths of skin friction for nothing, and angles near
# 90 deg divide by ~zero in the padding maths.
_ANGLE_BOUNDS = {"nose": (10.0, 80.0), "tail": (5.0, 45.0)}


@dataclass
class PayloadGrid:
    """A payload voxelised in a flow-aligned frame.

    ``rotation`` maps world coordinates into the frame whose +X axis is the
    wind direction; everything in this module works in that frame and rotates
    back only at the end.
    """

    occupancy: np.ndarray
    pitch: float
    origin: np.ndarray
    rotation: np.ndarray
    pad: np.ndarray = None  # padding in metres per axis, axis 0 along the flow
    distance_outside: np.ndarray = field(repr=False, default=None)
    distance_anisotropy: float | None = None
    warnings: list[str] = field(default_factory=list)

    def max_safe_radius(self, anisotropy: float, clearance: float = 0.0) -> float:
        """Largest closing radius the padding can hold.

        The anisotropic ball reaches ``anisotropy * r`` metres along the flow
        but only ``r`` across it. Exceed the padding and the dilation runs into
        the array border, where the erosion then shaves a slab off the end --
        silently, and the resulting fairing no longer contains the payload.
        """
        pad = np.asarray(self.pad, dtype=float)
        reach = np.array([max(anisotropy, 1e-6), 1.0, 1.0])
        return float(np.min((pad - clearance) / reach))

    @property
    def voxel_volume(self) -> float:
        return float(self.pitch**3)

    def to_world(self, points: np.ndarray) -> np.ndarray:
        """Voxel-index coordinates back to world space."""
        local = self.origin + np.asarray(points, dtype=float) * self.pitch
        return local @ self.rotation  # rotation is orthonormal, so R^T == inverse


def thinnest_feature(mesh: trimesh.Trimesh) -> float:
    """Smallest bounding-box dimension across the payload's separate bodies.

    Used to bound the voxel pitch so no part is lost to under-resolution.
    """
    try:
        bodies = mesh.split(only_watertight=False)
    except Exception:
        bodies = []
    if not bodies:
        bodies = [mesh]

    smallest = float("inf")
    for body in bodies:
        extents = np.asarray(body.extents, dtype=float)
        positive = extents[extents > 1e-9]
        if positive.size:
            smallest = min(smallest, float(positive.min()))
    return 0.0 if not np.isfinite(smallest) else smallest


def flow_frame(direction) -> np.ndarray:
    """Rotation taking world coordinates into a frame with +X along the wind."""
    x_axis = normalize(direction)
    reference = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(reference, x_axis))) > 0.9:
        reference = np.array([0.0, 1.0, 0.0])
    y_axis = normalize(np.cross(reference, x_axis))
    z_axis = np.cross(x_axis, y_axis)
    return np.vstack([x_axis, y_axis, z_axis])


def build_grid(
    mesh: trimesh.Trimesh,
    direction=(1.0, 0.0, 0.0),
    resolution: int = DEFAULT_RESOLUTION,
    margin: float = 0.4,
    pitch: float | None = None,
    anisotropy: float = DEFAULT_ANISOTROPY,
    streamline: tuple[float, float] | None = None,
    clearance: float = 0.0,
) -> PayloadGrid:
    """Voxelise ``mesh`` in the flow-aligned frame, with room to dilate into.

    ``margin`` is the padding around the payload as a fraction of its longest
    extent; it has to exceed the largest closing radius plus clearance or the
    dilation runs off the edge of the grid and the fairing comes out clipped.

    ``streamline`` is the (nose, tail) angle pair the envelope will be built
    with, if any. It matters here because the tail cone is *long* -- half the
    cross-flow width over tan(12 deg) is 2.4 body-widths -- and a grid padded
    symmetrically would clip it off. The padding this adds is asymmetric:
    exactly the streamwise room the cones can reach, and nothing spanwise.
    """
    rotation = flow_frame(direction)
    local = mesh.copy()
    # Rotating the mesh rather than the grid keeps the voxel array axis-aligned,
    # which is what makes the anisotropic distance transform a one-liner.
    local.apply_transform(np.vstack([np.hstack([rotation, np.zeros((3, 1))]), [0, 0, 0, 1]]))

    bounds = np.asarray(local.bounds, dtype=float)
    extents = bounds[1] - bounds[0]
    longest = float(np.max(extents))
    if longest <= 0:
        raise ValueError("Payload has zero extent")

    warnings: list[str] = []
    if pitch is None:
        pitch = longest / max(int(resolution), 16)
        # A part thinner than a few voxels can vanish or fuse with a neighbour,
        # which changes the component count and so corrupts the entire sweep.
        # A 50 mm wheel on a 2 m vehicle is exactly this case.
        thinnest = thinnest_feature(local)
        if 0.0 < thinnest < pitch * 3.0:
            pitch = thinnest / 3.0

    # Isotropic padding, with the radius clamped to what it can hold (see
    # max_safe_radius). Padding the flow axis by `anisotropy` times more would
    # let the sweep run further but inflates the grid several-fold, and the
    # radii that matter are far below the limit in practice.
    base_pad = max(longest * margin, pitch * 4)
    pad = np.array([base_pad, base_pad, base_pad])
    pad_lower = pad.copy()
    pad_upper = pad.copy()
    if streamline is not None:
        nose_room, tail_room = streamline_rooms(extents, streamline, clearance, pitch)
        pad_lower[0] += nose_room
        pad_upper[0] += tail_room
    origin = bounds[0] - pad_lower
    dims = np.ceil((extents + pad_lower + pad_upper) / pitch).astype(int) + 1

    if int(np.prod(dims)) > MAX_VOXELS:
        # Coarsen rather than fail: a clipped grid would silently produce a
        # truncated fairing, which is far worse than a blurrier one.
        scale = (int(np.prod(dims)) / MAX_VOXELS) ** (1.0 / 3.0)
        pitch *= scale
        dims = np.ceil((extents + pad_lower + pad_upper) / pitch).astype(int) + 1
        warnings.append(
            f"Voxel pitch was coarsened to {pitch * 1000:.1f} mm to stay within the memory "
            "budget; parts thinner than that may merge or disappear in the topology count."
        )

    voxels = local.voxelized(pitch=pitch)
    filled = np.asarray(voxels.matrix, dtype=bool)
    # Solid, not just a shell: closing a hollow shell would leave the interior
    # as a separate component and confuse the topology count.
    filled = ndimage.binary_fill_holes(filled)

    transform = np.asarray(voxels.transform, dtype=float)
    shell_origin = transform[:3, 3]
    offset = np.round((shell_origin - origin) / pitch).astype(int)

    occupancy = np.zeros(tuple(dims), dtype=bool)
    lower = np.clip(offset, 0, None)
    upper = np.minimum(offset + np.asarray(filled.shape), dims)
    source_lower = lower - offset
    source_upper = source_lower + (upper - lower)
    if np.any(upper <= lower):
        raise ValueError("Voxelisation fell outside the padded grid")
    occupancy[lower[0]:upper[0], lower[1]:upper[1], lower[2]:upper[2]] = filled[
        source_lower[0]:source_upper[0],
        source_lower[1]:source_upper[1],
        source_lower[2]:source_upper[2],
    ]

    if not occupancy.any():
        raise ValueError("Payload voxelised to nothing; try a finer resolution")

    _, bodies = ndimage.label(occupancy)
    mesh_bodies = len(mesh.split(only_watertight=False)) or 1
    if bodies < mesh_bodies:
        warnings.append(
            f"The STL has {mesh_bodies} separate bodies but only {bodies} survive voxelisation "
            f"at {pitch * 1000:.1f} mm; some parts are touching or too thin to separate."
        )

    return PayloadGrid(
        occupancy=occupancy,
        pitch=float(pitch),
        origin=origin,
        rotation=rotation,
        pad=pad,
        warnings=warnings,
    )


def _sampling(pitch: float, anisotropy: float) -> tuple[float, float, float]:
    """Voxel spacing for the distance transform, squashed along the flow.

    Axis 0 is the wind direction. Making it *shorter* means a streamwise gap of
    a given physical length registers as a smaller distance, so it closes at a
    smaller radius than the same gap across the flow.
    """
    return (pitch / max(anisotropy, 1e-6), pitch, pitch)


def _ensure_distance(grid: PayloadGrid, anisotropy: float) -> np.ndarray:
    """Distance-to-payload field, cached per (grid, anisotropy) pair."""
    if grid.distance_outside is None or grid.distance_anisotropy != anisotropy:
        grid.distance_outside = ndimage.distance_transform_edt(
            ~grid.occupancy, sampling=_sampling(grid.pitch, anisotropy)
        )
        grid.distance_anisotropy = anisotropy
    return grid.distance_outside


def closed_mask(
    grid: PayloadGrid,
    radius: float,
    anisotropy: float = DEFAULT_ANISOTROPY,
    distance_outside: np.ndarray | None = None,
    clearance: float = 0.0,
) -> np.ndarray:
    """Morphological closing of the payload by an ellipsoid of size ``radius``.

    Closing is erosion(dilation(S)). Both steps come from Euclidean distance
    transforms, which is O(n) per step rather than the cost of convolving with
    a large structuring element.

    With ``clearance`` set, the result is the *skin* -- the closed set grown
    outward by that gap. That outward offset is itself a dilation, so it merges
    any bodies closer than twice the clearance. Counting the closed set instead
    would report a topology the built fairing does not have.
    """
    sampling = _sampling(grid.pitch, anisotropy)
    if distance_outside is None:
        distance_outside = _ensure_distance(grid, anisotropy)

    if radius <= 0:
        closed = grid.occupancy.copy()
        return _grow(grid, closed, clearance) if clearance > 0 else closed

    dilated = distance_outside <= radius
    # Force an empty border. Without it the erosion's distance transform finds
    # no zero beyond the array edge, so voxels there survive, the closed set
    # runs off the grid and the isosurface comes out open instead of solid.
    dilated[0, :, :] = dilated[-1, :, :] = False
    dilated[:, 0, :] = dilated[:, -1, :] = False
    dilated[:, :, 0] = dilated[:, :, -1] = False

    # Erode the dilated set by the same ball: keep voxels further than r from
    # the outside of it.
    inner = ndimage.distance_transform_edt(dilated, sampling=sampling)
    closed = inner > radius
    return _grow(grid, closed, clearance) if clearance > 0 else closed


def _grow(grid: PayloadGrid, mask: np.ndarray, clearance: float) -> np.ndarray:
    """Offset a mask outward by ``clearance``, isotropically.

    Matches the level set build_fairing extracts, so the topology counted
    during the sweep is the topology of the skin that actually gets built.
    """
    distance = ndimage.distance_transform_edt(~mask, sampling=grid.pitch)
    return distance <= clearance


def _clamped_angles(nose_angle_deg: float, tail_angle_deg: float) -> tuple[float, float]:
    nose_low, nose_high = _ANGLE_BOUNDS["nose"]
    tail_low, tail_high = _ANGLE_BOUNDS["tail"]
    return (
        float(np.clip(nose_angle_deg, nose_low, nose_high)),
        float(np.clip(tail_angle_deg, tail_low, tail_high)),
    )


def streamline_rooms(
    extents, angles: tuple[float, float], clearance: float, pitch: float
) -> tuple[float, float]:
    """Streamwise padding the nose and tail cones need, in metres.

    A cone dies where the accumulated taper has eaten the section's inradius,
    and the inradius is bounded by half the smaller cross-flow extent plus the
    clearance the level set later adds outward. Anything short of this room
    would clip the tail tip -- silently, which is the one failure mode this
    module keeps having to design out.
    """
    nose_deg, tail_deg = _clamped_angles(*angles)
    half_cross = 0.5 * float(min(extents[1], extents[2])) + clearance
    nose_room = half_cross / float(np.tan(np.radians(nose_deg))) + clearance + 6 * pitch
    tail_room = half_cross / float(np.tan(np.radians(tail_deg))) + clearance + 6 * pitch
    return float(nose_room), float(tail_room)


def streamline_mask(
    mask: np.ndarray,
    pitch: float,
    nose_angle_deg: float = DEFAULT_NOSE_ANGLE_DEG,
    tail_angle_deg: float = DEFAULT_TAIL_ANGLE_DEG,
) -> np.ndarray:
    """The minimal taper-bounded envelope of ``mask``, axis 0 along the flow.

    The envelope is the union, over every cross-section of the payload, of
    that section swept downstream while eroding at tan(tail) per metre and
    upstream while eroding at tan(nose): each section casts a shallow cone
    aft and a steep one forward, and the body is the upper envelope of all of
    them. It is the smallest set that contains the payload, never grows
    steeper than the nose angle, and never shrinks steeper than the tail
    angle -- which is exactly the "minimum drag body that still fits" under
    the attached-flow heuristic, before the solvers get their say.

    Computed exactly, not by repeated binary erosion (whose per-slice step of
    tan(12 deg) * pitch is a fifth of a voxel and would round to nothing):
    per-slice Euclidean distance fields are carried along the axis with the
    taper subtracted per step and merged with a running max. Level sets of a
    max are unions of level sets, and level sets of (EDT - c) are exact
    erosions, so the positive set of the carry *is* the union of cones with
    no accumulated discretisation error.

    Two lumps in line fair into each other automatically -- the leading
    body's tail cone reaches the trailing body's nose -- so this can merge
    components the closing kept separate. That is the aerodynamically right
    call, but callers should recount bodies afterwards rather than trust the
    plateau's number.
    """
    nose_deg, tail_deg = _clamped_angles(nose_angle_deg, tail_angle_deg)
    envelope = np.zeros_like(mask)
    passes = (
        # Tail cones: swept downstream, shallow.
        (float(np.tan(np.radians(tail_deg))) * pitch, range(mask.shape[0])),
        # Nose cones: swept upstream, steep.
        (float(np.tan(np.radians(nose_deg))) * pitch, range(mask.shape[0] - 1, -1, -1)),
    )
    for step, order in passes:
        carry = np.full(mask.shape[1:], -np.inf)
        for index in order:
            carry -= step
            if mask[index].any():
                np.maximum(
                    carry,
                    ndimage.distance_transform_edt(mask[index], sampling=pitch),
                    out=carry,
                )
            envelope[index] |= carry > 0.0
    return envelope


@dataclass
class Plateau:
    """A range of closing radii over which the topology does not change."""

    components: int
    radius_min: float
    radius_max: float
    radius: float  # representative value, the middle of the plateau
    width: float

    def to_dict(self) -> dict:
        return {
            "components": self.components,
            "radius_min": self.radius_min,
            "radius_max": self.radius_max,
            "radius": self.radius,
            "width": self.width,
        }


@dataclass
class SweepResult:
    radii: list[float]
    components: list[int]
    plateaus: list[Plateau]
    pitch: float
    anisotropy: float
    payload_extents: list[float]

    def to_dict(self) -> dict:
        return {
            "radii": self.radii,
            "components": self.components,
            "plateaus": [item.to_dict() for item in self.plateaus],
            "pitch": self.pitch,
            "anisotropy": self.anisotropy,
            "payload_extents": self.payload_extents,
        }


def auto_radius_limit(grid: PayloadGrid, distance_outside: np.ndarray) -> float:
    """The radius at which the last gap inside the payload closes.

    This is the radius of the largest empty ball that fits inside the payload's
    own bounding box. Sweeping past it tells you nothing -- everything has
    already merged -- and sweeping a fixed fraction of the overall size instead
    puts every interesting transition below the first sample, which is how the
    first version of this missed the topology entirely.
    """
    filled = np.argwhere(grid.occupancy)
    lower = filled.min(axis=0)
    upper = filled.max(axis=0) + 1
    interior = distance_outside[lower[0]:upper[0], lower[1]:upper[1], lower[2]:upper[2]]
    largest_gap = float(interior.max()) if interior.size else 0.0
    return max(largest_gap * 1.15, grid.pitch * 8)


def safe_radii(grid: PayloadGrid, radii, anisotropy: float, clearance: float = 0.0) -> tuple[list[float], str | None]:
    """Drop radii the grid padding cannot support, and say so if any went."""
    limit = grid.max_safe_radius(anisotropy, clearance)
    kept = [float(value) for value in radii if value <= limit]
    if not kept:
        kept = [0.0]
    if len(kept) == len(list(radii)):
        return kept, None
    return kept, (
        f"Closing radii above {limit:.3g} m were dropped: the padded grid cannot hold a "
        f"dilation that large, and a clipped one would produce a fairing that does not "
        "enclose the payload."
    )


def sweep(
    grid: PayloadGrid,
    radii: list[float] | None = None,
    anisotropy: float = DEFAULT_ANISOTROPY,
    max_samples: int = 18,
    progress=None,
    clearance: float = 0.0,
) -> SweepResult:
    """Locate the topology transitions across closing radius.

    Component count is monotone non-increasing in r, so a uniform sweep wastes
    most of its samples confirming what is already merged. Instead: grow the
    upper bound until everything is one body, then bisect between neighbouring
    samples that disagree. Same budget, transitions located far more precisely,
    which is what sets the plateau edges.
    """
    extents = np.asarray(grid.occupancy.shape, dtype=float) * grid.pitch
    distance_outside = _ensure_distance(grid, anisotropy)
    limit = grid.max_safe_radius(anisotropy)
    tolerance = max(grid.pitch, limit / 128.0)

    samples: dict[float, int] = {}

    def count_at(radius: float) -> int:
        radius = float(min(max(radius, 0.0), limit))
        existing = next((key for key in samples if abs(key - radius) < 1e-12), None)
        if existing is not None:
            return samples[existing]
        mask = closed_mask(grid, radius, anisotropy, distance_outside=distance_outside, clearance=clearance)
        _, count = ndimage.label(mask)
        samples[radius] = int(count)
        if progress is not None:
            progress(
                {
                    "phase": "sweep",
                    "index": len(samples),
                    "total": max_samples,
                    "message": f"Closing radius {radius * 1000:.0f} mm gives {count} "
                    f"{'body' if count == 1 else 'bodies'}",
                }
            )
        return int(count)

    if radii is not None:
        for radius in radii:
            count_at(radius)
    else:
        count_at(0.0)
        # Grow until everything has merged, so the sweep spans exactly the
        # interesting range rather than a guessed fraction of the payload size.
        upper = max(limit / 16.0, grid.pitch * 2)
        while count_at(upper) > 1 and upper < limit and len(samples) < max_samples // 2:
            upper = min(upper * 2.0, limit)

        # Bisect the widest bracket that still straddles a transition.
        while len(samples) < max_samples:
            ordered = sorted(samples)
            bracket = None
            widest = 0.0
            for low, high in zip(ordered, ordered[1:]):
                if samples[low] != samples[high] and (high - low) > max(tolerance, widest):
                    widest = high - low
                    bracket = (low, high)
            if bracket is None:
                break
            count_at(0.5 * (bracket[0] + bracket[1]))

    ordered = sorted(samples)
    counts = [samples[key] for key in ordered]

    clipped_note = None
    if counts and counts[-1] > 1:
        clipped_note = (
            f"Even at the largest radius the grid can hold ({limit:.3g} m) the payload still "
            f"splits into {counts[-1]} bodies. A single-body fairing would need more padding."
        )
        if clipped_note not in grid.warnings:
            grid.warnings.append(clipped_note)

    return SweepResult(
        radii=[float(value) for value in ordered],
        components=counts,
        plateaus=find_plateaus(ordered, counts),
        pitch=grid.pitch,
        anisotropy=anisotropy,
        payload_extents=extents.tolist(),
    )


def find_plateaus(
    radii,
    counts,
    max_bodies: int = MAX_SENSIBLE_BODIES,
    open_top: bool = True,
) -> list[Plateau]:
    """Runs of constant component count, widest first.

    Plateau width is a confidence measure: a topology that survives a wide
    range of radii is a real design, while one that exists only in a narrow
    band is an artefact of two lumps happening to be nearly touching.
    """
    radii = [float(value) for value in radii]
    counts = [int(value) for value in counts]
    if not radii:
        return []

    plateaus: list[Plateau] = []
    start = 0
    for index in range(1, len(counts) + 1):
        if index < len(counts) and counts[index] == counts[start]:
            continue

        low = radii[start]
        # The plateau really runs up to the next sampled radius, where the
        # count changed; use that as the upper edge.
        high = radii[index] if index < len(radii) else radii[-1]
        if index >= len(radii) and open_top:
            # The final topology persists for every larger radius, so its
            # sampled upper edge is an artefact of where the sweep stopped.
            # Cap it at twice the transition: past that the fairing is just an
            # inflated blob, not a different design.
            high = max(low * 2.0, high)
        count = counts[start]
        if 0 < count <= max_bodies:
            plateaus.append(
                Plateau(
                    components=count,
                    radius_min=low,
                    radius_max=high,
                    radius=0.5 * (low + high),
                    width=high - low,
                )
            )
        start = index

    # Keep one entry per topology: the widest run wins if a count reappears.
    best: dict[int, Plateau] = {}
    for plateau in plateaus:
        current = best.get(plateau.components)
        if current is None or plateau.width > current.width:
            best[plateau.components] = plateau

    return sorted(best.values(), key=lambda item: (-item.width, item.components))


def build_fairing(
    grid: PayloadGrid,
    radius: float,
    clearance: float = 0.02,
    anisotropy: float = DEFAULT_ANISOTROPY,
    smoothing_iterations: int = 12,
    streamline: tuple[float, float] | None = None,
    field_sigma_voxels: float = 1.2,
) -> trimesh.Trimesh:
    """Build the fairing skin for one closing radius.

    The surface is the ``clearance`` level set of the signed distance field of
    the closed payload, so the gap between payload and skin is exact and the
    skin is smooth to sub-voxel precision rather than following voxel corners.

    With ``streamline`` set to a (nose, tail) angle pair, the closed set is
    replaced by its taper-bounded envelope first: same topology decision, but
    a profile shaped to fly rather than merely to enclose.

    ``field_sigma_voxels`` smooths the distance *field*, not the mesh. The
    field is computed from a binary mask, so its level sets carry staircase
    ripples at the voxel pitch -- which is exactly the wavelength Taubin on
    the extracted mesh is too local to remove. A Gaussian on the field kills
    them at the source: it preserves linear fields exactly, so wherever the
    surface is flat or gently curved the level set does not move at all, and
    only features at the noise scale are touched. The sigma is capped well
    below the extraction level so tight corners are rounded by millimetres,
    never eaten; the containment check downstream still verifies the result.
    """
    mask = closed_mask(grid, radius, anisotropy, distance_outside=_ensure_distance(grid, anisotropy))
    if streamline is not None:
        mask = streamline_mask(mask, grid.pitch, *streamline)
        # The rooms added in build_grid keep the cones clear of the border;
        # clearing it anyway means a mis-sized grid yields a closed (if
        # clipped) surface instead of an open mesh that fails everywhere
        # downstream.
        mask[0, :, :] = mask[-1, :, :] = False
        mask[:, 0, :] = mask[:, -1, :] = False
        mask[:, :, 0] = mask[:, :, -1] = False

    # Isotropic distance here: the anisotropy was a decision rule for what
    # merges, not a distortion we want baked into the skin.
    outside = ndimage.distance_transform_edt(~mask, sampling=grid.pitch)
    inside = ndimage.distance_transform_edt(mask, sampling=grid.pitch)
    field = (outside - inside).astype(np.float32)
    del outside, inside  # each is as large as the grid; drop them before filtering

    level = max(float(clearance), grid.pitch * 0.5)

    if field_sigma_voxels > 0:
        # Cap the filter width against the extraction level: the level set of
        # a smoothed field shifts by about sigma^2 * curvature / 2, and the
        # tightest curvature on an offset surface is 1/level (the rounded
        # payload edges). At 0.6 * level the worst-case shift is under a fifth
        # of the clearance; at the default sigma and pitch it is millimetres.
        sigma = min(field_sigma_voxels * grid.pitch, 0.6 * level) / grid.pitch
        field = ndimage.gaussian_filter(field, sigma=sigma)

    if float(field.max()) <= level:
        raise ValueError("Clearance exceeds the padding around the payload")

    from skimage import measure

    vertices, faces, _, _ = measure.marching_cubes(field, level=level, spacing=(1.0, 1.0, 1.0))
    mesh = trimesh.Trimesh(vertices=grid.to_world(vertices), faces=faces, process=True)

    # Marching cubes orients faces along the field gradient, which points into
    # the body here, so the winding comes out inside-out and every containment
    # test would answer backwards. Fix it off the volume sign before smoothing.
    if mesh.is_watertight and mesh.volume < 0:
        mesh.invert()

    if smoothing_iterations > 0:
        # Taubin rather than Laplacian: it smooths without the steady shrinkage
        # that would eat into the clearance we just established.
        trimesh.smoothing.filter_taubin(mesh, iterations=int(smoothing_iterations))

    mesh.remove_unreferenced_vertices()
    return mesh


@dataclass
class Candidate:
    """One proposed fairing, ready to be costed by the solvers."""

    radius: float
    components: int
    plateau_width: float
    clearance: float
    frontal_area: float
    wetted_area: float
    volume: float | None
    watertight: bool
    contains_payload: bool | None
    triangle_count: int
    min_gap: float | None = None
    gap_ratio: float | None = None
    streamlined: bool = False
    nose_angle_deg: float | None = None
    tail_angle_deg: float | None = None
    mesh: trimesh.Trimesh = field(repr=False, default=None)

    @property
    def choked(self) -> bool:
        """Are two bodies close enough that the flow between them chokes?

        A narrow gap is high velocity with a boundary layer off both walls,
        often worse than the solid bridge you were avoiding -- and it needs
        cells finer than the gap, which makes the case expensive to mesh as
        well as bad to fly.
        """
        return self.gap_ratio is not None and self.gap_ratio < CHOKED_GAP_RATIO

    def to_dict(self) -> dict:
        return {
            "radius": self.radius,
            "components": self.components,
            "plateau_width": self.plateau_width,
            "clearance": self.clearance,
            "frontal_area": self.frontal_area,
            "wetted_area": self.wetted_area,
            "volume": self.volume,
            "watertight": self.watertight,
            "contains_payload": self.contains_payload,
            "triangle_count": self.triangle_count,
            "min_gap": self.min_gap,
            "gap_ratio": self.gap_ratio,
            "choked": self.choked,
            "streamlined": self.streamlined,
            "nose_angle_deg": self.nose_angle_deg,
            "tail_angle_deg": self.tail_angle_deg,
        }


def smallest_gap(bodies: list[trimesh.Trimesh]) -> float | None:
    """Closest approach between any two separate bodies, or None if only one.

    Vertex-to-vertex via a KD-tree: an over-estimate of the true surface gap
    by at most the triangle size, which is plenty for deciding whether a
    channel is dangerously narrow.
    """
    if len(bodies) < 2:
        return None

    from scipy.spatial import cKDTree

    smallest = float("inf")
    for index, body in enumerate(bodies):
        tree = cKDTree(np.asarray(body.vertices, dtype=float))
        for other in bodies[index + 1:]:
            distances, _ = tree.query(np.asarray(other.vertices, dtype=float), k=1)
            smallest = min(smallest, float(np.min(distances)))
    return None if not np.isfinite(smallest) else smallest


def check_containment(fairing: trimesh.Trimesh, payload: trimesh.Trimesh, samples: int = 4000) -> bool | None:
    """Does the fairing actually enclose the payload?

    Guaranteed by construction -- closing is extensive and the level set sits
    a clearance outside it -- but smoothing runs afterwards, so this verifies
    rather than assumes.

    Returns None when the check could not run. That distinction matters: an
    earlier version caught every exception and returned False, so a missing
    ray-casting backend was reported to the user as "the payload does not
    fit", which is the most damaging thing this function could get wrong.
    """
    if not fairing.is_watertight:
        return False
    points = np.asarray(payload.vertices, dtype=float)
    if len(points) > samples:
        step = max(len(points) // samples, 1)
        points = points[::step]
    try:
        return bool(np.all(fairing.contains(points)))
    except Exception:
        return None


def candidates_from_sweep(
    grid: PayloadGrid,
    payload: trimesh.Trimesh,
    result: SweepResult,
    direction=(1.0, 0.0, 0.0),
    clearance: float = 0.02,
    smoothing_iterations: int = 12,
    limit: int = 4,
    progress=None,
    build_grid_override: PayloadGrid | None = None,
    streamline: tuple[float, float] | None = None,
) -> list[Candidate]:
    """Turn the widest plateaus into buildable fairings with their metrics.

    The sweep runs on a coarse grid because counting components tolerates a
    blurry payload; the skins are built on a finer one, since that geometry is
    what the solvers actually mesh. Radii are physical lengths, so they carry
    across unchanged.

    With ``streamline`` set, every skin is the taper-bounded envelope rather
    than the raw closing. The plateau still decides what merges *spanwise*;
    the envelope handles the streamwise profile, and may fair in-line bodies
    into each other, so the component count is recounted from the mesh.
    """
    grid = build_grid_override or grid
    angles = _clamped_angles(*streamline) if streamline is not None else None
    candidates: list[Candidate] = []
    for index, plateau in enumerate(result.plateaus[:limit]):
        if progress is not None:
            progress(
                {
                    "phase": "candidate",
                    "index": index,
                    "total": min(len(result.plateaus), limit),
                    "message": f"Building the {plateau.components}-body fairing "
                    f"(r = {plateau.radius:.4g} m)",
                }
            )
        try:
            mesh = build_fairing(
                grid,
                plateau.radius,
                clearance=clearance,
                anisotropy=result.anisotropy,
                smoothing_iterations=smoothing_iterations,
                streamline=angles,
            )
        except Exception:
            continue

        bodies = mesh.split(only_watertight=False)
        if angles is not None and len(bodies) < plateau.components and progress is not None:
            progress(
                {
                    "phase": "candidate",
                    "index": index,
                    "total": min(len(result.plateaus), limit),
                    "message": f"Tail fairing merged the {plateau.components} bodies "
                    f"into {len(bodies)}",
                }
            )
        min_gap = smallest_gap(bodies)
        body_scale = float(np.max(np.asarray(mesh.extents, dtype=float)))
        candidates.append(
            Candidate(
                min_gap=min_gap,
                gap_ratio=None if min_gap is None or body_scale <= 0 else min_gap / body_scale,
                radius=plateau.radius,
                components=max(len(bodies), 1),
                plateau_width=plateau.width,
                clearance=clearance,
                frontal_area=frontal_area(mesh, direction),
                wetted_area=float(mesh.area),
                volume=float(mesh.volume) if mesh.is_watertight else None,
                watertight=bool(mesh.is_watertight),
                contains_payload=check_containment(mesh, payload),
                triangle_count=int(len(mesh.faces)),
                streamlined=angles is not None,
                nose_angle_deg=None if angles is None else angles[0],
                tail_angle_deg=None if angles is None else angles[1],
                mesh=mesh,
            )
        )

    return candidates
