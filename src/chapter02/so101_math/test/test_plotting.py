from matplotlib import rcParams

from so101_math.plotting import configure_chinese_font


def test_plotting_font_configuration():
    selected = configure_chinese_font()
    assert rcParams["axes.unicode_minus"] is False
    if selected is not None:
        assert rcParams["font.sans-serif"][0] == selected
