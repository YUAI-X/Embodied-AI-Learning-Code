from matplotlib import rcParams
# 作者：宇哥的具身笔记


from so101_math.plotting import configure_chinese_font


def test_plotting_font_configuration():
    selected = configure_chinese_font()
    assert rcParams["axes.unicode_minus"] is False
    if selected is not None:
        assert rcParams["font.sans-serif"][0] == selected
