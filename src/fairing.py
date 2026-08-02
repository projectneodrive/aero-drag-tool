"""Automatic fairing generation from an arbitrary payload STL.

The fairing is **always a single closed shell**. Separate pods around separate
lumps are not offered: the gap between two pods is a channel with a boundary
layer off both walls, which is usually worse than the solid bridge it was
avoiding and needs cells finer than the gap to mesh at all. So the only
question left is *how much closing it takes* to reach one body -- a length
scale, not a choice.

A morphological closing (dilate by r, then erode by r) hugs each lump at small
r and bridges the gaps at large r, and the topology changes on its own. The
number of connected components is monotone non-increasing in r, so there is a
single threshold radius where the payload first becomes one body. This module
locates that threshold by bisection and builds the shell just above it: enough
closing to merge, and no more, because every millimetre past the threshold is
frontal area bought for nothing.

Two refinements matter:

* The closing is **anisotropic**, elongated along the flow. Two lumps in line
  should merge much more readily than two lumps side by side: bridging in-line
  lumps costs almost no frontal area and removes a wake-impingement problem,
  while bridging side-by-side lumps means filling the whole span between them.
* The surface comes from an isosurface of a **signed distance field** at a
  positive level, not from the raw voxel mask. That gives a smooth, sub-voxel
  skin and makes the clearance gap exact by construction, so the payload is
  guaranteed to fit with room to spare.

The closing decides *whether it is one body*, not the *profile*. It is bounded above by the
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

# Safety margin on the merge threshold. The sweep runs on a coarse grid and the
# skin is built on a fine one, which resolves narrow gaps the sweep blurred
# shut, so building exactly at the threshold can come out as two bodies.
MERGE_MARGIN = 1.06

# How much to open the radius each time a built shell still splits, and how
# many times to try before giving up.
MERGE_ESCALATION = 1.45
MERGE_ATTEMPTS = 4

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

# How far a blended shoulder spreads along the flow, as a fraction of the
# payload's cross-flow half-width. Zero reproduces the faceted envelope
# exactly -- the minimal one, whose shoulders are creases -- so the two
# profiles are one family, not two code paths, and the search can walk
# continuously from one to the other.
#
# Quoted as a *length* rather than as the fillet radius, because a radius does
# not mean the same thing at both ends: an arc of radius r turning through the
# taper angle only spans r*sin(angle) streamwise, so a radius that visibly
# softens a 45 deg nose is invisible on a 12 deg tail. A length blends both
# shoulders over comparable distances, which is what "smooth transition"
# actually asks for. The radius each shoulder needs follows from it.
DEFAULT_SHOULDER_BLEND = 0.0
_BLEND_BOUNDS = (0.0, 1.5)

# How many tangent panels approximate each blended shoulder. The fillet is the
# lower envelope of these, so more panels is a closer arc at one running-max
# pass each; 8 spans the quarter turn in 12.9 deg steps, whose worst deviation
# from the true arc is 1/cos(6.4 deg) - 1 = 0.6% of the fillet radius -- well
# inside a voxel on any grid this module builds.
SHOULDER_TANGENTS = 8


def blend_bounds() -> tuple[float, float]:
    """Validity bounds for the shoulder blend length, as a half-width fraction.

    Zero is the faceted envelope. Past the upper bound the fillet is longer
    than the body is wide and the two shoulders start to run into each other,
    where the construction stops meaning much.
    """
    return _BLEND_BOUNDS


def angle_bounds(kind: str) -> tuple[float, float]:
    """Validity bounds for a taper angle in degrees, ``kind`` nose or tail.

    Anything outside these is clamped by the envelope builder, so a search
    over the angles has to know them to avoid spending solves on candidates
    that all come back as the same clamped shape.
    """
    return _ANGLE_BOUNDS[kind]


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
    shoulder_blend: float = DEFAULT_SHOULDER_BLEND,
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
        nose_room, tail_room = streamline_rooms(
            extents, streamline, clearance, pitch, shoulder_blend
        )
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


def _clamped_blend(blend: float) -> float:
    return float(np.clip(blend, *_BLEND_BOUNDS))


def streamline_rooms(
    extents,
    angles: tuple[float, float],
    clearance: float,
    pitch: float,
    blend: float = DEFAULT_SHOULDER_BLEND,
) -> tuple[float, float]:
    """Streamwise padding the nose and tail cones need, in metres.

    A cone dies where the accumulated taper has eaten the section's inradius,
    and the inradius is bounded by half the smaller cross-flow extent plus the
    clearance the level set later adds outward. Anything short of this room
    would clip the tail tip -- silently, which is the one failure mode this
    module keeps having to design out.

    A blended shoulder pushes the taper downstream before it reaches full
    rate, so the cone dies ``blend_radius * tan(angle)`` further out. That
    term is small on a shallow tail and the whole fillet radius on a 45 degree
    nose, which is exactly the asymmetry the fillet has: rounding a sharper
    corner moves more.
    """
    nose_deg, tail_deg = _clamped_angles(*angles)
    half_cross = 0.5 * float(min(extents[1], extents[2])) + clearance
    blend_length = _clamped_blend(blend) * half_cross

    def room(degrees: float) -> float:
        taper = float(np.tan(np.radians(degrees)))
        # The blended profile asymptotes to the bare taper offset outward by
        # `gap`, so it dies `gap / taper` further along -- which works out to
        # the blend length times hypot(1, taper), a touch over the blend
        # length on a shallow tail and 1.41x it on a 45 degree nose.
        reach = fillet_radius(taper, blend_length) * taper
        return half_cross / taper + reach + clearance + 6 * pitch

    return float(room(nose_deg)), float(room(tail_deg))


def fillet_radius(taper: float, blend_length: float) -> float:
    """Fillet radius that spreads a shoulder over ``blend_length`` streamwise.

    The shoulder turns through atan(taper), and an arc of radius r turning
    through that angle covers r*sin(atan(taper)) along the flow. Inverting
    that is the whole conversion.
    """
    if blend_length <= 0.0 or taper <= 0.0:
        return 0.0
    return float(blend_length * np.hypot(1.0, taper) / taper)


def shoulder_tangents(
    taper: float, radius: float, pitch: float, count: int = SHOULDER_TANGENTS
) -> list[tuple[float, float]]:
    """Tangent lines whose lower envelope is the filleted taper profile.

    In the (streamwise, radius) plane the taper limit is a straight line, and
    the minimal envelope's shoulder is the corner where that line meets the
    payload's own section: the profile's slope jumps from zero to the full
    taper across a single slice. Rounding that corner at ``radius`` replaces
    the line with the hyperbola tangent to the flat at the shoulder and
    asymptotic to the same taper downstream, so the section still never
    shrinks faster than the limit -- it just stops starting to discontinuously.

        r(d) = gap - sqrt((taper * d)^2 + gap^2),   gap = radius * taper^2

    The trick that makes this nearly free: a concave curve is the infimum of
    its own tangents, and each tangent is a *straight taper with an outward
    offset* -- which the running-max recursion already computes exactly. So
    the fillet costs one extra pass per tangent and needs no new machinery,
    and the panels it produces are what a real fairing is made of anyway.

    Parametrised by the tangent angle, which is the arc's own parameter: the
    taper scales as sin and the offset as 1 - cos, sampled uniformly over the
    quarter turn from flat to full rate. Returns (per-slice step, offset) in
    metres, ready to subtract from and add to a carry.
    """
    if radius <= 0.0 or count < 2:
        return [(taper * pitch, 0.0)]
    gap = radius * taper * taper
    turn = 0.5 * np.pi * np.arange(count) / (count - 1)
    return [
        (float(taper * np.sin(angle) * pitch), float(gap * (1.0 - np.cos(angle))))
        for angle in turn
    ]


def _cross_half_width(mask: np.ndarray, pitch: float) -> float:
    """Half the smaller cross-flow extent of ``mask``, in metres.

    The fillet radius is quoted as a fraction of this, so a blend setting
    means the same shape on any payload at any scale.
    """
    shadow = mask.any(axis=0)
    occupied = np.argwhere(shadow)
    if not occupied.size:
        return 0.0
    span = (occupied.max(axis=0) - occupied.min(axis=0) + 1) * pitch
    return 0.5 * float(np.min(span))


def streamline_mask(
    mask: np.ndarray,
    pitch: float,
    nose_angle_deg: float = DEFAULT_NOSE_ANGLE_DEG,
    tail_angle_deg: float = DEFAULT_TAIL_ANGLE_DEG,
    blend: float = DEFAULT_SHOULDER_BLEND,
) -> np.ndarray:
    """The taper-bounded envelope of ``mask``, axis 0 along the flow.

    The envelope is the union, over every cross-section of the payload, of
    that section swept downstream while eroding at tan(tail) per metre and
    upstream while eroding at tan(nose): each section casts a shallow cone
    aft and a steep one forward, and the body is the upper envelope of all of
    them. At ``blend`` zero it is the smallest set that contains the payload,
    never grows steeper than the nose angle, and never shrinks steeper than
    the tail angle -- the "minimum drag body that still fits" under the
    attached-flow heuristic, before the solvers get their say.

    Computed exactly, not by repeated binary erosion (whose per-slice step of
    tan(12 deg) * pitch is a fifth of a voxel and would round to nothing):
    per-slice Euclidean distance fields are carried along the axis with the
    taper subtracted per step and merged with a running max. Level sets of a
    max are unions of level sets, and level sets of (EDT - c) are exact
    erosions, so the positive set of the carry *is* the union of cones with
    no accumulated discretisation error.

    ``blend`` rounds the shoulders where the cones meet the payload's own
    section, as a fraction of the cross-flow half-width. That corner is a
    crease at blend zero, and unavoidably so: minimality forces the envelope
    to hold full section right up to the payload's last slice and to be
    shrinking at the full taper one slice later. **Minimality and tangent
    continuity are in direct conflict**, and minimum volume is not what this
    tool is optimising -- Cd.A is, and the frontal area it is normalised on is
    untouched here (see below). So the fillet is offered as a profile rather
    than assumed away, and the solver decides what it is worth.

    Each shoulder is filleted by casting the cones from a *family* of tangent
    tapers instead of one (see ``shoulder_tangents``) and taking the tightest.
    Two properties fall out of that construction, and both are worth having:

    * **Containment survives.** Every tangent envelope is pointwise at least
      the bare-taper envelope, so their minimum is too: a blended shell holds
      whatever the faceted one held, and the check downstream still verifies.
    * **Frontal area is unchanged.** The flat tangent (zero taper, zero
      offset) is in the family, so the blend can never exceed the running
      maximum of the payload's own sections. The fillet is bought in wetted
      area and length, never in silhouette -- which is what makes it a fair
      question to put to a solver that ranks on Cd.A.

    Two lumps in line fair into each other automatically -- the leading
    body's tail cone reaches the trailing body's nose -- so this can merge
    components the closing kept separate. That is the aerodynamically right
    call, but callers should recount bodies afterwards rather than trust the
    plateau's number.
    """
    nose_deg, tail_deg = _clamped_angles(nose_angle_deg, tail_angle_deg)
    blend_length = _clamped_blend(blend) * _cross_half_width(mask, pitch)

    envelope = np.zeros_like(mask)
    passes = (
        # Tail cones: swept downstream, shallow.
        (float(np.tan(np.radians(tail_deg))), range(mask.shape[0])),
        # Nose cones: swept upstream, steep.
        (float(np.tan(np.radians(nose_deg))), range(mask.shape[0] - 1, -1, -1)),
    )
    for taper, order in passes:
        tangents = shoulder_tangents(taper, fillet_radius(taper, blend_length), pitch)
        carries = [np.full(mask.shape[1:], -np.inf) for _ in tangents]
        blended = np.empty(mask.shape[1:], dtype=float)
        for index in order:
            for carry, (step, _) in zip(carries, tangents):
                carry -= step
            if mask[index].any():
                section = ndimage.distance_transform_edt(mask[index], sampling=pitch)
                for carry in carries:
                    np.maximum(carry, section, out=carry)
            # The fillet is the *lower* envelope of the tangents: each is a
            # taper the section is permitted to follow, and the body takes
            # the tightest of them. With one tangent this is the bare cone
            # and the whole loop collapses to the original recursion.
            np.add(carries[0], tangents[0][1], out=blended)
            for carry, (_, offset) in zip(carries[1:], tangents[1:]):
                np.minimum(blended, carry + offset, out=blended)
            envelope[index] |= blended > 0.0
    return envelope


@dataclass
class SweepResult:
    """Component count against closing radius, and where it reaches one body.

    ``merge_radius`` is the smallest sampled radius at which the skin is a
    single connected body -- the number the shell is built from. It is None
    when even the largest radius the grid can hold leaves the payload in
    pieces, which is a padding problem rather than a shape one.
    """

    radii: list[float]
    components: list[int]
    merge_radius: float | None
    pitch: float
    anisotropy: float
    payload_extents: list[float]
    limit: float = 0.0

    @property
    def bodies_at_zero(self) -> int:
        return self.components[0] if self.components else 1

    def to_dict(self) -> dict:
        return {
            "radii": self.radii,
            "components": self.components,
            "merge_radius": self.merge_radius,
            "bodies_at_zero": self.bodies_at_zero,
            "pitch": self.pitch,
            "anisotropy": self.anisotropy,
            "payload_extents": self.payload_extents,
            "limit": self.limit,
        }


def sweep(
    grid: PayloadGrid,
    anisotropy: float = DEFAULT_ANISOTROPY,
    max_samples: int = 16,
    progress=None,
    clearance: float = 0.0,
) -> SweepResult:
    """Find the smallest closing radius that makes the payload one body.

    Component count is monotone non-increasing in r, which is what makes this
    a bisection rather than a search: there is exactly one threshold, and
    every radius above it also merges. So grow an upper bound until the count
    reaches one, then halve the bracket until it is tighter than a voxel.

    Sampling uniformly instead would spend most of its budget confirming what
    is already merged, and would locate the threshold no better than the
    sample spacing -- which then goes straight into the frontal area, since
    the shell is built just above it.
    """
    extents = np.asarray(grid.occupancy.shape, dtype=float) * grid.pitch
    distance_outside = _ensure_distance(grid, anisotropy)
    limit = grid.max_safe_radius(anisotropy, clearance)
    tolerance = max(grid.pitch, limit / 256.0)

    samples: dict[float, int] = {}

    def count_at(radius: float) -> int:
        radius = float(min(max(radius, 0.0), limit))
        existing = next((key for key in samples if abs(key - radius) < 1e-12), None)
        if existing is not None:
            return samples[existing]
        mask = closed_mask(
            grid, radius, anisotropy, distance_outside=distance_outside, clearance=clearance
        )
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

    merge_radius: float | None = None

    if count_at(0.0) == 1:
        # Already one lump: the shell is the clearance offset and nothing more.
        merge_radius = 0.0
    else:
        low = 0.0
        high = None
        upper = max(limit / 16.0, grid.pitch * 2)
        while len(samples) < max_samples:
            if count_at(upper) == 1:
                high = upper
                break
            low = upper
            if upper >= limit:
                break
            upper = min(upper * 2.0, limit)

        if high is not None:
            # Tighten the bracket. Every iteration keeps `low` at more than one
            # body and `high` at exactly one, so `high` is always a radius that
            # is known to work.
            while high - low > tolerance and len(samples) < max_samples:
                middle = 0.5 * (low + high)
                if count_at(middle) == 1:
                    high = middle
                else:
                    low = middle
            merge_radius = high

    ordered = sorted(samples)
    counts = [samples[key] for key in ordered]

    if merge_radius is None:
        note = (
            f"Even at the largest radius the grid can hold ({limit:.3g} m) the payload still "
            f"splits into {counts[-1]} bodies, so no single-body shell could be built. "
            "Raise the streamwise bias or the clearance, which both bridge gaps sooner."
        )
        if note not in grid.warnings:
            grid.warnings.append(note)

    return SweepResult(
        radii=[float(value) for value in ordered],
        components=counts,
        merge_radius=merge_radius,
        pitch=grid.pitch,
        anisotropy=anisotropy,
        payload_extents=extents.tolist(),
        limit=float(limit),
    )


def build_fairing(
    grid: PayloadGrid,
    radius: float,
    clearance: float = 0.02,
    anisotropy: float = DEFAULT_ANISOTROPY,
    smoothing_iterations: int = 12,
    streamline: tuple[float, float] | None = None,
    field_sigma_voxels: float = 1.2,
    shoulder_blend: float = DEFAULT_SHOULDER_BLEND,
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
        mask = streamline_mask(mask, grid.pitch, *streamline, blend=shoulder_blend)
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
class Shell:
    """The single-body fairing built around a payload."""

    radius: float
    merge_radius: float | None
    clearance: float
    anisotropy: float
    frontal_area: float
    wetted_area: float
    volume: float | None
    watertight: bool
    contains_payload: bool | None
    triangle_count: int
    # How many radii had to be tried before the built skin came out as one
    # body. More than one means the fine grid disagreed with the sweep grid.
    attempts: int = 1
    bodies: int = 1
    streamlined: bool = False
    nose_angle_deg: float | None = None
    tail_angle_deg: float | None = None
    # Shoulder fillet as a fraction of the cross-flow half-width. Zero is the
    # faceted envelope, whose shoulders are creases.
    shoulder_blend: float = DEFAULT_SHOULDER_BLEND
    mesh: trimesh.Trimesh = field(repr=False, default=None)

    def to_dict(self) -> dict:
        return {
            "radius": self.radius,
            "merge_radius": self.merge_radius,
            "clearance": self.clearance,
            "anisotropy": self.anisotropy,
            "frontal_area": self.frontal_area,
            "wetted_area": self.wetted_area,
            "volume": self.volume,
            "watertight": self.watertight,
            "contains_payload": self.contains_payload,
            "triangle_count": self.triangle_count,
            "attempts": self.attempts,
            "bodies": self.bodies,
            "streamlined": self.streamlined,
            "nose_angle_deg": self.nose_angle_deg,
            "tail_angle_deg": self.tail_angle_deg,
            "shoulder_blend": self.shoulder_blend,
        }


def smallest_gap(bodies: list[trimesh.Trimesh]) -> float | None:
    """Closest approach between any two separate bodies, or None if only one.

    Vertex-to-vertex via a KD-tree: an over-estimate of the true surface gap
    by at most the triangle size. The generated shell is always one body, so
    this is for meshing an imported STL that is not -- SU2 sizes its cells off
    the narrowest channel it has to resolve.
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


def build_single_shell(
    grid: PayloadGrid,
    payload: trimesh.Trimesh,
    result: SweepResult,
    direction=(1.0, 0.0, 0.0),
    clearance: float = 0.02,
    smoothing_iterations: int = 12,
    progress=None,
    build_grid_override: PayloadGrid | None = None,
    streamline: tuple[float, float] | None = None,
    shoulder_blend: float = DEFAULT_SHOULDER_BLEND,
) -> Shell:
    """Build the one-body shell at the smallest radius that merges the payload.

    The sweep runs on a coarse grid, because counting components tolerates a
    blurry payload; the skin is built on a finer one, since that geometry is
    what the solvers actually mesh. Radii are physical lengths, so they carry
    across unchanged -- but the fine grid *resolves narrow gaps the coarse one
    blurred shut*, so the threshold it found can come out as two bodies once
    built. That is why this verifies the result and opens the radius until the
    skin really is one body, rather than trusting the sweep's number.

    Failing to check is the whole failure mode this function exists to
    prevent: a two-piece "single shell" meshes into a choked channel and
    reports a drag coefficient for a shape nobody chose.
    """
    grid = build_grid_override or grid
    angles = _clamped_angles(*streamline) if streamline is not None else None
    blend = _clamped_blend(shoulder_blend) if angles is not None else DEFAULT_SHOULDER_BLEND
    limit = grid.max_safe_radius(result.anisotropy, clearance)

    if result.merge_radius is None:
        raise ValueError(
            "The payload never closes into a single body within the grid the padding can "
            "hold. Raise the streamwise bias or the clearance."
        )

    radius = min(result.merge_radius * MERGE_MARGIN, limit)
    mesh = None
    bodies = 0
    attempt = 0

    for attempt in range(1, MERGE_ATTEMPTS + 1):
        if progress is not None:
            progress(
                {
                    "phase": "shell",
                    "index": attempt - 1,
                    "total": MERGE_ATTEMPTS,
                    "message": f"Building the shell at r = {radius * 1000:.0f} mm"
                    + (f" (attempt {attempt})" if attempt > 1 else ""),
                }
            )
        mesh = build_fairing(
            grid,
            radius,
            clearance=clearance,
            anisotropy=result.anisotropy,
            smoothing_iterations=smoothing_iterations,
            streamline=angles,
            shoulder_blend=blend,
        )
        bodies = max(len(mesh.split(only_watertight=False)), 1)
        if bodies == 1:
            break
        if radius >= limit:
            break
        radius = min(radius * MERGE_ESCALATION, limit)
        if progress is not None:
            progress(
                {
                    "phase": "shell",
                    "message": f"The skin came out as {bodies} bodies at that radius; "
                    f"opening it to {radius * 1000:.0f} mm",
                }
            )

    return Shell(
        radius=float(radius),
        merge_radius=result.merge_radius,
        clearance=clearance,
        anisotropy=result.anisotropy,
        frontal_area=frontal_area(mesh, direction),
        wetted_area=float(mesh.area),
        volume=float(mesh.volume) if mesh.is_watertight else None,
        watertight=bool(mesh.is_watertight),
        contains_payload=check_containment(mesh, payload),
        triangle_count=int(len(mesh.faces)),
        attempts=attempt,
        bodies=bodies,
        streamlined=angles is not None,
        nose_angle_deg=None if angles is None else angles[0],
        tail_angle_deg=None if angles is None else angles[1],
        shoulder_blend=blend,
        mesh=mesh,
    )
