from pathlib import Path
import json
import math
import shutil
import sys


ROOT = Path(__file__).resolve().parents[1]
PKGS = ROOT / "work" / "cq_v7_pkgs"
sys.path.insert(0, str(PKGS))

import cadquery as cq
from PIL import Image, ImageDraw, ImageFont


OUT = Path(__file__).resolve().parent
V6 = ROOT / "grip_v6_clickfit_spring_variants"
V6_FULL = V6 / "full_set"
FULL = OUT / "full_set"
FULL.mkdir(parents=True, exist_ok=True)


# Coordinate convention for the revised case:
#   +X short end: four vibrator cables
#   -X short end: fully closed
#   -Y long side: pressure-sensor cable
#   +Y long side: regulated-power cable
# The V6 outer envelope remains 80 x 57 x 60 mm, so the V6 lid fit is retained.
VIBRATOR_NOTCH_W = 46.0
VIBRATOR_NOTCH_BOTTOM_Z = 12.0
PRESSURE_NOTCH_W = 18.0
POWER_NOTCH_W = 14.0
SIDE_NOTCH_CENTER_X = 24.0
SIDE_NOTCH_BOTTOM_Z = 12.0
LID_CENTER_OPENING_W = 22.0
LID_CENTER_OPENING_D = 15.0
LID_CENTER_OPENING_CENTER_X = 17.0
LID_CENTER_OPENING_CENTER_Y = -10.0


def box(w, d, h, x=0.0, y=0.0, z=0.0):
    return (
        cq.Workplane("XY")
        .box(w, d, h, centered=(True, True, False))
        .translate((x, y, z))
    )


def build_case():
    case = cq.importers.importStep(str(V6_FULL / "case_base.step"))

    # V6 had four large, open-top wall gaps. Restore those wall areas first,
    # with a small overlap so the unions are robust, then cut only the cable
    # exits actually needed by the assembled electronics.
    fills = [
        box(2.2, 32.4, 48.2, x=-38.9, z=11.8),
        box(2.2, 32.4, 48.2, x=+38.9, z=11.8),
        box(48.4, 2.2, 48.2, y=-27.4, z=11.8),
        box(48.4, 2.2, 48.2, y=+27.4, z=11.8),
    ]
    for fill in fills:
        case = case.union(fill)

    # Four vibrator leads: an open-top notch, widened from V6's 32 mm to
    # 46 mm. The 12 mm lower rail remains for stiffness.
    case = case.cut(
        box(
            3.4,
            VIBRATOR_NOTCH_W,
            49.0,
            x=39.0,
            z=VIBRATOR_NOTCH_BOTTOM_Z,
        )
    )

    # The short end opposite the vibrators stays fully closed. Only the two
    # required cable regions remain open on the long sides, both moved close
    # to the vibrator end as requested.
    case = case.cut(
        box(
            PRESSURE_NOTCH_W,
            3.4,
            49.0,
            x=SIDE_NOTCH_CENTER_X,
            y=-27.5,
            z=SIDE_NOTCH_BOTTOM_Z,
        )
    )
    case = case.cut(
        box(
            POWER_NOTCH_W,
            3.4,
            49.0,
            x=SIDE_NOTCH_CENTER_X,
            y=+27.5,
            z=SIDE_NOTCH_BOTTOM_Z,
        )
    )
    return case.clean()


def build_lid():
    lid = cq.importers.importStep(str(V6_FULL / "case_lid_print.step"))

    # Close the former 36 x 20 mm centre opening, then recut the relocated
    # service opening. The requested position overlaps the middle-right and
    # lower-right narrow V6 slots, so both are closed first to preserve an
    # exact 22 x 15 mm rectangle. The other four narrow slots remain unchanged.
    lid = lid.union(box(36.4, 20.4, 2.0, z=0.0))
    lid = lid.union(box(4.4, 9.4, 2.0, x=29.0, y=0.0, z=0.0))
    lid = lid.union(box(4.4, 9.4, 2.0, x=29.0, y=-15.0, z=0.0))
    lid = lid.cut(
        box(
            LID_CENTER_OPENING_W,
            LID_CENTER_OPENING_D,
            3.0,
            x=LID_CENTER_OPENING_CENTER_X,
            y=LID_CENTER_OPENING_CENTER_Y,
            z=-0.5,
        )
    )
    return lid.clean()


def export_part(part, stem, folder=OUT):
    assert part.val().isValid(), stem
    assert len(part.solids().vals()) == 1, (stem, len(part.solids().vals()))
    cq.exporters.export(part, str(folder / f"{stem}.step"))
    cq.exporters.export(
        part,
        str(folder / f"{stem}.stl"),
        tolerance=0.04,
        angularTolerance=0.1,
    )


def bbox_data(part):
    b = part.val().BoundingBox()
    return [round(b.xlen, 3), round(b.ylen, 3), round(b.zlen, 3)]


def render(part, draw, cx, cy, scale, color):
    verts, tris = part.val().tessellate(0.18)

    def proj(v):
        return (
            cx + scale * (0.82 * v.x - 0.57 * v.y),
            cy + scale * (0.28 * v.x + 0.40 * v.y - 0.87 * v.z),
        )

    def depth(tri):
        return sum(
            0.47 * verts[i].x + 0.68 * verts[i].y + 0.56 * verts[i].z
            for i in tri
        )

    for tri in sorted(tris, key=depth):
        a, b, c = [verts[i] for i in tri]
        u = (b.x - a.x, b.y - a.y, b.z - a.z)
        v = (c.x - a.x, c.y - a.y, c.z - a.z)
        n = (
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        )
        norm = math.sqrt(sum(q * q for q in n)) or 1.0
        shade = 0.60 + 0.36 * abs(
            (n[0] * 0.25 + n[1] * 0.35 + n[2] * 0.90) / norm
        )
        draw.polygon(
            [proj(verts[i]) for i in tri],
            fill=tuple(int(q * shade) for q in color),
        )


def make_preview(case, lid):
    img = Image.new("RGB", (1600, 980), "#f4f6f8")
    draw = ImageDraw.Draw(img)
    font = ImageFont.load_default()

    draw.text((48, 32), "V7 CASE - WIRING CLEARANCE REVISION", fill="#17202a", font=font)
    draw.text(
        (48, 58),
        "V6 grip/click-fit retained | case outer size retained: 80 x 57 x 60 mm",
        fill="#334155",
        font=font,
    )
    render(case, draw, 440, 660, 6.0, (92, 145, 178))
    render(lid.translate((0, 0, 25)), draw, 1130, 480, 6.0, (170, 185, 196))

    # Compact side-opening map, using the same coordinate convention as CAD.
    ox, oy, sx, sy = 1010, 690, 5.0, 5.0
    draw.rectangle(
        [ox - 40 * sx, oy - 28.5 * sy, ox + 40 * sx, oy + 28.5 * sy],
        outline="#1f2937",
        width=3,
    )
    draw.line(
        [ox + 40 * sx, oy - 23 * sy, ox + 40 * sx, oy + 23 * sy],
        fill="#ef6c00",
        width=8,
    )
    draw.line(
        [ox + 15 * sx, oy - 28.5 * sy, ox + 33 * sx, oy - 28.5 * sy],
        fill="#0284c7",
        width=8,
    )
    draw.line(
        [ox + 17 * sx, oy + 28.5 * sy, ox + 31 * sx, oy + 28.5 * sy],
        fill="#16a34a",
        width=8,
    )
    draw.text((1240, 668), "+X: vibrator 46 mm", fill="#ef6c00", font=font)
    draw.text((740, 790), "-X: fully closed", fill="#334155", font=font)
    draw.text((1030, 525), "-Y: pressure 18 mm", fill="#0284c7", font=font)
    draw.text((1030, 850), "+Y: power 14 mm", fill="#16a34a", font=font)
    draw.text((900, 930), "Lid opening: 22 x 15 mm, centre X=+17 / Y=+10 mm", fill="#334155", font=font)
    img.save(OUT / "preview_case_v7.png")


case = build_case()
lid = build_lid()
export_part(case, "case_base_v7")
export_part(lid, "case_lid_v7")

# Case-only print plate: both parts remain in their original support-free print
# orientation and fit on a 180 x 180 mm bed.
case_pair = cq.Compound.makeCompound(
    [
        case.translate((-46.0, 0.0, 0.0)).val(),
        lid.translate((46.0, 0.0, 0.0)).val(),
    ]
)
assert len(case_pair.Solids()) == 2
cq.exporters.export(
    case_pair,
    str(OUT / "PRINT_CASE_V7_2_parts.stl"),
    tolerance=0.04,
    angularTolerance=0.1,
)
cq.exporters.export(case_pair, str(OUT / "PRINT_CASE_V7_2_parts.step"))

# Preserve all V6 grip-side files and provide a complete V7 nine-part plate.
unchanged = (
    "plate_left",
    "plate_right",
    "spring_2.0mm_clickfit",
    "support_pad",
)
for name in unchanged:
    for ext in ("step", "stl"):
        shutil.copy2(V6_FULL / f"{name}.{ext}", FULL / f"{name}.{ext}")
shutil.copy2(OUT / "case_base_v7.step", FULL / "case_base_v7.step")
shutil.copy2(OUT / "case_base_v7.stl", FULL / "case_base_v7.stl")
shutil.copy2(OUT / "case_lid_v7.step", FULL / "case_lid_v7.step")
shutil.copy2(OUT / "case_lid_v7.stl", FULL / "case_lid_v7.stl")

parts = {
    "case_base_v7": case,
    "case_lid_v7": lid,
    "plate_left": cq.importers.importStep(str(V6_FULL / "plate_left.step")),
    "plate_right": cq.importers.importStep(str(V6_FULL / "plate_right.step")),
    "spring_2.0mm_clickfit": cq.importers.importStep(
        str(V6_FULL / "spring_2.0mm_clickfit.step")
    ),
    "support_pad": cq.importers.importStep(str(V6_FULL / "support_pad.step")),
}
placements = [
    ("case_base_v7", -45.0, -56.5),
    ("case_lid_v7", 42.4, -54.1),
    ("plate_left", -64.5, 18.0),
    ("plate_right", -24.5, 18.0),
    ("spring_2.0mm_clickfit", 30.0, 15.0),
    ("support_pad", 65.0, 5.0),
    ("support_pad", 77.0, 5.0),
    ("support_pad", 65.0, 17.0),
    ("support_pad", 77.0, 17.0),
]
placed = []
for name, x, y in placements:
    item = parts[name].translate((x, y, 0.0))
    b = item.val().BoundingBox()
    assert b.xmin >= -85.01 and b.xmax <= 85.01, (name, b.xmin, b.xmax)
    assert b.ymin >= -85.01 and b.ymax <= 85.01, (name, b.ymin, b.ymax)
    for other in placed:
        assert item.intersect(other).val().Volume() < 1e-6
    placed.append(item)

full_batch = cq.Compound.makeCompound([item.val() for item in placed])
assert len(full_batch.Solids()) == 9
cq.exporters.export(
    full_batch,
    str(FULL / "PRINT_ALL_9_parts_v7.stl"),
    tolerance=0.04,
    angularTolerance=0.1,
)
cq.exporters.export(full_batch, str(FULL / "print_layout_9_parts_v7.step"))

checks = {
    "version": 7,
    "case_outer_size_mm": bbox_data(case),
    "lid_outer_size_mm": bbox_data(lid),
    "case_valid_solid": case.val().isValid(),
    "lid_valid_solid": lid.val().isValid(),
    "case_volume_mm3": round(case.val().Volume(), 3),
    "lid_volume_mm3": round(lid.val().Volume(), 3),
    "openings": {
        "vibrator_open_top_notch_width_mm": VIBRATOR_NOTCH_W,
        "opposite_short_end": "fully closed",
        "pressure_sensor_open_top_notch_width_mm": PRESSURE_NOTCH_W,
        "regulated_power_open_top_notch_width_mm": POWER_NOTCH_W,
        "long_side_notch_center_x_mm": SIDE_NOTCH_CENTER_X,
        "lid_center_opening_mm": [LID_CENTER_OPENING_W, LID_CENTER_OPENING_D],
        "lid_opening_center_x_mm": LID_CENTER_OPENING_CENTER_X,
        "lid_opening_center_y_mm": LID_CENTER_OPENING_CENTER_Y,
    },
    "parts_in_case_print": len(case_pair.Solids()),
    "parts_in_full_print": len(full_batch.Solids()),
    "unchanged_from_v6": [
        "outer case envelope and lid fit",
        "plate_left",
        "plate_right",
        "2.0 mm click-fit spring",
        "four support pads",
        "four narrow lid slots (middle-right and lower-right closed)",
    ],
}
(OUT / "checks.json").write_text(
    json.dumps(checks, ensure_ascii=False, indent=2), encoding="utf-8"
)
make_preview(case, lid)
print(json.dumps(checks, ensure_ascii=False, indent=2))
