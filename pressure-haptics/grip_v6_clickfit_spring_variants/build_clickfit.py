from pathlib import Path
import json
import math
import shutil
import sys

PKGS = Path(__file__).parent / "cadquery_pkgs"
if (PKGS / "cadquery" / "__init__.py").is_file():
    sys.path.insert(0, str(PKGS))

import cadquery as cq
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "grip_v6_clickfit_spring_variants"
FULL = OUT / "full_set"
OUT.mkdir(parents=True, exist_ok=True)
FULL.mkdir(parents=True, exist_ok=True)

SOURCE = Path(r"C:\Users\yokei\Downloads\grip_v4_sensor_space\grip_v4")

# Click-fit dimensions in mm. The spring tongue is split so the two halves can
# flex inward while the 0.45 mm side bumps pass the narrow entrance.
SOCKET_ENTRANCE_W = 4.15
SOCKET_CHAMBER_W = 4.60
SOCKET_H = 6.20
SOCKET_DEPTH = 6.20
CHAMBER_START = 4.00
TONGUE_W = 3.80
TONGUE_H = 5.80
TONGUE_DEPTH = 6.00
SPLIT_W = 1.40
BUMP_PROJECTION = 0.30


def box(w, d, h, x=0, y=0, z=0, centered=(True, True, False)):
    return cq.Workplane("XY").box(w, d, h, centered=centered).translate((x, y, z))


def spring_core(t):
    p = cq.Workplane("XY").moveTo(-12, -7).lineTo(-12, 7)
    for i in range(6):
        x = -12 + 4 * i
        top = i % 2 == 0
        p = p.threePointArc(
            (x + 2, 9 if top else -9),
            (x + 4, 7 if top else -7),
        ).lineTo(x + 4, -7 if top else 7)
    return cq.Workplane("XZ", origin=(-12, -7, 0)).rect(t, 6).sweep(p.wire())


def wedge(points, height=TONGUE_H):
    return cq.Workplane("XY").polyline(points).close().extrude(height / 2, both=True)


def click_connector(side, cy):
    """Connector in assembled spring coordinates; side is -1 (left) or +1."""
    # Shoulder overlaps the swept spring end and remains outside the socket.
    shoulder_cx = side * 13.25
    shoulder = cq.Workplane("XY").box(3.5, TONGUE_W, TONGUE_H).translate(
        (shoulder_cx, cy, 0)
    )

    # Five millimetre split tongue, with a solid 0.8 mm root.
    tongue_cx = side * 18.00
    tongue = cq.Workplane("XY").box(TONGUE_DEPTH + 0.4, TONGUE_W, TONGUE_H).translate(
        (tongue_cx, cy, 0)
    )
    slit_cx = side * 18.60
    tongue = tongue.cut(
        cq.Workplane("XY").box(5.0, SPLIT_W, TONGUE_H + 0.4).translate(
            (slit_cx, cy, 0)
        )
    )

    # Symmetric ramps make both insertion and intentional removal possible.
    if side < 0:
        xs = (-21.0, -19.8, -18.6)
    else:
        xs = (21.0, 19.8, 18.6)
    upper = wedge(
        [(xs[0], cy + TONGUE_W / 2),
         (xs[1], cy + TONGUE_W / 2 + BUMP_PROJECTION),
         (xs[2], cy + TONGUE_W / 2)]
    )
    lower = wedge(
        [(xs[0], cy - TONGUE_W / 2),
         (xs[1], cy - TONGUE_W / 2 - BUMP_PROJECTION),
         (xs[2], cy - TONGUE_W / 2)]
    )
    return shoulder.union(tongue).union(upper).union(lower)


def spring(t=1.2):
    return (
        spring_core(t)
        .union(click_connector(-1, -7))
        .union(click_connector(+1, +7))
    )


def click_socket(center_x):
    # Narrow guide channel plus a wider chamber for the retaining bumps.
    entrance = box(
        SOCKET_ENTRANCE_W, SOCKET_H, SOCKET_DEPTH,
        x=center_x, y=0, z=0,
    )
    chamber = box(
        SOCKET_CHAMBER_W, SOCKET_H, SOCKET_DEPTH - CHAMBER_START,
        x=center_x, y=0, z=CHAMBER_START,
    )
    # A shallow lead-in compensates for first-layer spread and guides insertion.
    lead = box(
        SOCKET_ENTRANCE_W + 0.35, SOCKET_H + 0.35, 0.70,
        x=center_x, y=0, z=0,
    )
    return entrance.union(chamber).union(lead)


def plate(socket_x):
    p = box(31, 64, 8).edges("|Z").fillet(2)
    for yy in (-20, 20):
        p = p.cut(box(25.6, 17.0, 5.5, y=yy, z=2.5))
        for xx in (-14.5, 14.5):
            p = p.cut(box(5, 14, 5.5, x=xx, y=yy, z=2.5))
    return p.cut(click_socket(socket_x))


SPRING_THICKNESSES = (1.8, 2.0, 2.2, 2.4)
parts = {
    "plate_left": plate(7),
    "plate_right": plate(-7),
}
for thickness in SPRING_THICKNESSES:
    parts[f"spring_{thickness:.1f}mm_clickfit"] = spring(thickness).translate((0, 0, 3))

stats = {}
for name, solid in parts.items():
    assert solid.val().isValid(), name
    assert len(solid.solids().vals()) == 1, (name, len(solid.solids().vals()))
    cq.exporters.export(solid, str(OUT / f"{name}.step"))
    cq.exporters.export(
        solid, str(OUT / f"{name}.stl"), tolerance=0.04, angularTolerance=0.1
    )
    b = solid.val().BoundingBox()
    stats[name] = {
        "size_mm": [b.xlen, b.ylen, b.zlen],
        "volume_mm3": solid.val().Volume(),
    }

# Assembly follows the original V4 orientation.
left = (
    parts["plate_left"]
    .rotate((0, 0, 0), (0, 1, 0), -90)
    .rotate((0, 0, 0), (1, 0, 0), 90)
    .translate((-15, 0, 32))
)
right = (
    parts["plate_right"]
    .rotate((0, 0, 0), (0, 1, 0), 90)
    .rotate((0, 0, 0), (1, 0, 0), 90)
    .translate((15, 0, 32))
    .rotate((15, 0, 32), (16, 0, 32), 180)
)
assembled_spring = spring(2.0).translate((0, 0, 32))
for side in (left, right):
    overlap = side.intersect(assembled_spring).val().Volume()
    assert overlap < 1e-4, overlap

assembly = cq.Assembly()
assembly.add(left, name="left", color=cq.Color(0.25, 0.50, 0.70))
assembly.add(right, name="right", color=cq.Color(0.25, 0.50, 0.70))
assembly.add(assembled_spring, name="spring_clickfit", color=cq.Color(0.95, 0.65, 0.15))
assembly.save(str(OUT / "grip_assembly_clickfit_2.0mm.step"))

# Both plates and the new spring in print orientation.
grip_print = cq.Compound.makeCompound([
    parts["plate_left"].translate((-42, 0, 0)).val(),
    parts["plate_right"].translate((0, 0, 0)).val(),
    parts["spring_2.0mm_clickfit"].translate((42, 0, 0)).val(),
])
assert len(grip_print.Solids()) == 3
cq.exporters.export(
    grip_print, str(OUT / "PRINT_GRIP_3_parts_2.0mm.stl"),
    tolerance=0.04, angularTolerance=0.1,
)
cq.exporters.export(grip_print, str(OUT / "PRINT_GRIP_3_parts_2.0mm.step"))

# Four springs arranged from softest to stiffest, left to right.
spring_test = cq.Compound.makeCompound([
    parts[f"spring_{thickness:.1f}mm_clickfit"]
    .rotate((0, 0, 0), (0, 0, 1), 90)
    .translate(((-36 + i * 24), 0, 0)).val()
    for i, thickness in enumerate(SPRING_THICKNESSES)
])
assert len(spring_test.Solids()) == 4
cq.exporters.export(
    spring_test, str(OUT / "PRINT_SPRING_TEST_4_variants.stl"),
    tolerance=0.04, angularTolerance=0.1,
)
cq.exporters.export(spring_test, str(OUT / "PRINT_SPRING_TEST_4_variants.step"))

# Quick fit test: one real socket and one real connector, isolated from the full parts.
test_socket = box(12, 12, 8).cut(click_socket(0))
test_tab = click_connector(-1, 0).translate((16, 0, 3))
cq.exporters.export(test_socket, str(OUT / "fit_test_socket.stl"), tolerance=0.04)
cq.exporters.export(test_tab, str(OUT / "fit_test_tab.stl"), tolerance=0.04)
fit_pair = cq.Compound.makeCompound([
    test_socket.translate((-8, 0, 0)).val(),
    test_tab.translate((4, 0, 0)).val(),
])
cq.exporters.export(fit_pair, str(OUT / "PRINT_FIT_TEST_2_parts.stl"), tolerance=0.04)

# Rebuild the previous full print set with the new plates and required new spring.
case = cq.importers.importStep(str(SOURCE / "full_set" / "case_base.step"))
lid = cq.importers.importStep(str(SOURCE / "full_set" / "case_lid_print.step"))
pad = cq.importers.importStep(str(SOURCE / "full_set" / "support_pad.step"))
full_parts = {
    "case_base": case,
    "case_lid_print": lid,
    "plate_left": parts["plate_left"],
    "plate_right": parts["plate_right"],
    "spring_2.0mm_clickfit": parts["spring_2.0mm_clickfit"],
    "support_pad": pad,
}
for name in ("case_base", "case_lid_print", "support_pad"):
    shutil.copy2(SOURCE / "full_set" / f"{name}.step", FULL / f"{name}.step")
    shutil.copy2(SOURCE / "full_set" / f"{name}.stl", FULL / f"{name}.stl")
for name in ("plate_left", "plate_right", "spring_2.0mm_clickfit"):
    shutil.copy2(OUT / f"{name}.step", FULL / f"{name}.step")
    shutil.copy2(OUT / f"{name}.stl", FULL / f"{name}.stl")

placements = [
    ("case_base", -45.0, -56.5),
    ("case_lid_print", 42.4, -54.1),
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
    item = full_parts[name].translate((x, y, 0))
    b = item.val().BoundingBox()
    assert b.xmin >= -85.01 and b.xmax <= 85.01, (name, b.xmin, b.xmax)
    assert b.ymin >= -85.01 and b.ymax <= 85.01, (name, b.ymin, b.ymax)
    for other in placed:
        assert item.intersect(other).val().Volume() < 1e-6
    placed.append(item)
full_batch = cq.Compound.makeCompound([p.val() for p in placed])
assert len(full_batch.Solids()) == 9
cq.exporters.export(
    full_batch, str(FULL / "PRINT_ALL_9_parts.stl"),
    tolerance=0.04, angularTolerance=0.1,
)
cq.exporters.export(full_batch, str(FULL / "print_layout_9_parts.step"))

stats["click_fit"] = {
    "socket_entrance_width_mm": SOCKET_ENTRANCE_W,
    "socket_chamber_width_mm": SOCKET_CHAMBER_W,
    "socket_height_mm": SOCKET_H,
    "socket_depth_mm": SOCKET_DEPTH,
    "tongue_width_mm": TONGUE_W,
    "tongue_height_mm": TONGUE_H,
    "tongue_depth_mm": TONGUE_DEPTH,
    "split_width_mm": SPLIT_W,
    "retaining_bump_each_side_mm": BUMP_PROJECTION,
    "nominal_entrance_side_clearance_mm": (SOCKET_ENTRANCE_W - TONGUE_W) / 2,
    "parts_in_grip_print": 3,
    "parts_in_full_print": 9,
    "spring_test_thicknesses_mm": list(SPRING_THICKNESSES),
    "recommended_start_mm": 2.0,
}
(OUT / "checks.json").write_text(json.dumps(stats, indent=2), encoding="utf-8")

readme = """V6: 着脱式クリックフィット＋ばね太さ比較版

おすすめ手順
1. 最初に PRINT_FIT_TEST_2_parts.stl を100%で印刷する。
2. テスト片を差し込み、軽いクリック感で止まり、手で引いて外せることを確認する。
3. ばねの感触を比較する場合は PRINT_SPRING_TEST_4_variants.stl を100%で印刷する。
   配置は左から1.8 / 2.0 / 2.2 / 2.4 mm。プレートから外す前に油性ペン等で太さを書く。
4. 2.0 mmばねと握り部分をまとめて交換する場合は PRINT_GRIP_3_parts_2.0mm.stl を印刷する。
5. ケースも含めて全部印刷する場合だけ full_set/PRINT_ALL_9_parts.stl を使う。ここには2.0 mmばねを収録。

変更点
- 21 x 19 mmの圧力センサー貼付面と、握りパーツ全長64 mmはV4のまま。
- ばね端を二股の弾性タブに変更。
- タブ両側に0.30 mmの傾斜付き保持突起を追加。
- 受け穴は入口4.15 mm、奥4.60 mmの二段形状。奥で突起が戻って保持する。
- 差込深さを3.3 mmから6.2 mmへ増やし、横揺れも減らした。
- 挿入と取り外しの両方向に傾斜があるため、接着せず繰り返し着脱できる。
- しなる蛇行部分だけを1.8 / 2.0 / 2.2 / 2.4 mmの4段階で用意。
- 接続タブは全種類で同じ寸法なので、同じ左右プレートへ交換して比較できる。

使い方
- ばね端を穴にまっすぐ押し込み、クリック感が出るまで差し込む。
- 外すときは握りパーツを押さえ、ばねの端に近い太い部分をまっすぐ引く。
- 蛇行している薄い部分を強くねじらない。

印刷
- PLA想定、0.4 mmノズル、積層0.20 mm、壁3周以上。
- サポートは通常不要。STLの向きを変更せず100%で印刷。
- 初層が強く潰れる設定では受け穴が狭くなるため、テスト片で先に確認。

注意
太さを上げるほど押し込み量は減り、親指側と人差し指側の振動も伝わりにくくなる一方、必要な指の力は増える。
プリンター、材料、温度で嵌合と硬さは変わるため、実機での着脱耐久と振動絶縁は未検証。
硬すぎる場合は無理に押し込まず、受け穴入口のバリだけを軽く除去する。
"""
(OUT / "README.txt").write_text(readme, encoding="utf-8")


def render(part, draw, cx, cy, scale, color):
    verts, tris = part.val().tessellate(0.12)
    def proj(v):
        return (cx + scale * (0.866 * v.x - 0.5 * v.y),
                cy + scale * (0.25 * v.x + 0.433 * v.y - 0.866 * v.z))
    def depth(tri):
        return sum(0.433 * verts[i].x + 0.75 * verts[i].y + 0.5 * verts[i].z for i in tri)
    for tri in sorted(tris, key=depth):
        a, b, c = [verts[i] for i in tri]
        u = (b.x-a.x, b.y-a.y, b.z-a.z)
        v = (c.x-a.x, c.y-a.y, c.z-a.z)
        n = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        norm = math.sqrt(sum(q*q for q in n)) or 1
        shade = 0.64 + 0.30 * abs((n[0]*0.3+n[1]*0.4+n[2]*0.866)/norm)
        draw.polygon([proj(verts[i]) for i in tri], fill=tuple(int(q*shade) for q in color))


img = Image.new("RGB", (1300, 760), "#f4f7fa")
draw = ImageDraw.Draw(img)
render(left, draw, 420, 590, 5.1, (65, 135, 177))
render(right, draw, 420, 590, 5.1, (65, 135, 177))
render(assembled_spring, draw, 420, 590, 5.1, (239, 160, 45))
render(parts["spring_2.0mm_clickfit"], draw, 980, 425, 8.0, (239, 160, 45))
draw.text((55, 35), "V6 CLICK-FIT GRIP - 2.0 mm ASSEMBLY", fill=(25, 45, 60))
draw.text((55, 65), "21 x 19 mm sensor area retained | 64 mm plates", fill=(25, 45, 60))
draw.text((740, 80), "1.8 / 2.0 / 2.2 / 2.4 mm springs included", fill=(25, 45, 60))
draw.text((740, 105), "Push to click; pull straight to remove", fill=(25, 45, 60))
img.save(OUT / "preview_clickfit.png")

shutil.copy2(Path(__file__), OUT / "build_clickfit.py")
print(json.dumps(stats, indent=2))
