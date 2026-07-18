"""Add a Chinese explanatory layer to the truthful V12 CAD render."""

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "exploded.png"
TARGET = ROOT / "exploded-annotated.png"
FONT = "/System/Library/Fonts/STHeiti Medium.ttc"

NAVY = "#10243D"
BLUE = "#1769D2"
BLUE_LIGHT = "#EAF3FF"
GRAY = "#526476"
LINE = "#8DB8ED"
WHITE = "#FFFFFF"


def font(size: int, bold: bool = False):
    # STHeiti is a stable system CJK face on this macOS installation.
    return ImageFont.truetype(FONT, size=size)


def draw_multiline_center(draw, box, text, face, fill, spacing=6):
    x0, y0, x1, y1 = box
    bbox = draw.multiline_textbbox((0, 0), text, font=face, spacing=spacing, align="center")
    width = bbox[2] - bbox[0]
    height = bbox[3] - bbox[1]
    draw.multiline_text(
        ((x0 + x1 - width) / 2, (y0 + y1 - height) / 2 - bbox[1]),
        text,
        font=face,
        fill=fill,
        spacing=spacing,
        align="center",
    )


def main():
    src = Image.open(SOURCE).convert("RGB")
    canvas = Image.new("RGB", (2400, 1700), "#F7F9FC")
    draw = ImageDraw.Draw(canvas)

    title_font = font(68)
    subtitle_font = font(32)
    label_font = font(28)
    small_font = font(24)
    motion_font = font(27)

    draw.text((80, 54), "键盘 / 妙控板联动支架｜真实 CAD 结构拆解", font=title_font, fill=NAVY, stroke_width=1)
    draw.text((84, 142), "仅移动原装配实体进行展示 · 未增加 AI 补画零件 · 左右结构对称", font=subtitle_font, fill=GRAY)
    draw.line((80, 198, 2320, 198), fill=BLUE, width=4)

    # Preserve the CAD render without repainting it.  The original contains a
    # generous white margin, so a crop makes the actual mechanism readable.
    crop = src.crop((300, 170, 1320, 1040))
    crop = crop.resize((1580, 1348), Image.Resampling.LANCZOS)
    image_origin = (30, 225)
    canvas.paste(crop, image_origin)

    # Convert source-render pixel coordinates to the cropped image on canvas.
    def pt(x, y):
        return (
            image_origin[0] + (x - 300) * 1580 / 1020,
            image_origin[1] + (y - 170) * 1348 / 870,
        )

    callouts = [
        (1, "完整连续妙控板托盘", pt(840, 350), 250),
        (2, "完整连续键盘托盘", pt(800, 590), 405),
        (3, "妙控板长槽滑架", pt(1125, 530), 560),
        (4, "键盘交叉滚轮滑架", pt(1120, 670), 715),
        (5, "604ZZ 双侧斜槽导向", pt(1175, 625), 870),
        (6, "MR84ZZ 双滚轮\n滑动交点", pt(1090, 650), 1025),
        (7, "MISUMI CFS0.2\n恒力自动回位", pt(1145, 705), 1180),
        (8, "开放式镂空底座\n六点防滑", pt(820, 845), 1335),
    ]

    box_x0, box_x1 = 1680, 2320
    box_h = 118
    for number, label, target, y in callouts:
        box = (box_x0, y, box_x1, y + box_h)
        draw.rounded_rectangle(box, radius=18, fill=WHITE, outline=LINE, width=3)
        badge = (box_x0, y, box_x0 + 104, y + box_h)
        draw.rounded_rectangle(badge, radius=18, fill=BLUE)
        # Square off the badge's right edge while retaining the outer radius.
        draw.rectangle((box_x0 + 82, y, box_x0 + 104, y + box_h), fill=BLUE)
        draw_multiline_center(draw, badge, f"{number:02d}", font(38), WHITE)
        text_box = (box_x0 + 120, y + 6, box_x1 - 16, y + box_h - 6)
        draw_multiline_center(draw, text_box, label, label_font, NAVY, spacing=3)

        tx, ty = target
        elbow_x = 1635
        anchor_y = y + box_h / 2
        draw.line((tx, ty, elbow_x, anchor_y, box_x0, anchor_y), fill=BLUE, width=4, joint="curve")
        draw.ellipse((tx - 10, ty - 10, tx + 10, ty + 10), fill=WHITE, outline=BLUE, width=4)

    # Compact color legend tied directly to the CAD material colors.
    legend_y = 1515
    legend = [
        ("#3F8FD2", "妙控板运动组"),
        ("#D94A3A", "键盘运动组"),
        ("#3E4650", "固定结构"),
        ("#D2A62E", "轴承 / 滚轮"),
    ]
    lx = 80
    for color, text in legend:
        draw.rounded_rectangle((lx, legend_y, lx + 34, legend_y + 34), radius=7, fill=color)
        draw.text((lx + 48, legend_y - 1), text, font=small_font, fill=GRAY)
        lx += 300

    # Bottom motion chain is a functional explanation, not an exploded part.
    motion_box = (80, 1580, 2320, 1670)
    draw.rounded_rectangle(motion_box, radius=22, fill=BLUE_LIGHT, outline=LINE, width=2)
    motion = "按下键盘托  →  交叉滚轮沿蓝色长槽滑动  →  妙控板托向右手区域前移并下降  →  松手后恒力弹簧自动回位"
    draw_multiline_center(draw, motion_box, motion, motion_font, NAVY)

    canvas.save(TARGET, optimize=True)
    print(TARGET)


if __name__ == "__main__":
    main()
