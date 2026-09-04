"""Matplotlib中文字体配置，避免坐标轴和标题显示为方框。"""
# 作者：宇哥的具身笔记


from matplotlib import font_manager, rcParams


CHINESE_FONT_CANDIDATES = (
    "Noto Sans CJK SC",
    "Noto Sans CJK JP",
    "Source Han Sans SC",
    "Microsoft YaHei",
    "SimHei",
    "WenQuanYi Zen Hei",
    "Droid Sans Fallback",
    "AR PL UKai CN",
    "AR PL UMing CN",
)


def configure_chinese_font() -> str | None:
    """选择系统中第一个可用中文字体，并修复负号显示。

    不硬编码某台电脑的字体文件路径，使脚本可在不同Linux和Windows环境运行。
    返回实际选中的字体名；没有找到中文字体时返回None。
    """
    available = {font.name for font in font_manager.fontManager.ttflist}
    selected = next(
        (name for name in CHINESE_FONT_CANDIDATES if name in available), None
    )
    if selected is not None:
        rcParams["font.family"] = "sans-serif"
        rcParams["font.sans-serif"] = [selected, "DejaVu Sans"]
    # 默认Unicode负号在部分中文字体中缺失，使用普通减号更稳妥。
    rcParams["axes.unicode_minus"] = False
    return selected
