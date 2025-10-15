# 这是一个将sad的格式的数据转换成我们需要马尔科夫格式的程序
"""
sad 格式的 encoder 数据为 [odom time left_pulse right_pulse]
马尔科夫的数据为 [odom time ? s θ ?]
需要将 left_pulse 和 right_pulse 转换成 s 和 θ
转换的公式为 s = v * t = w * r * t
"""
def convert_data_type():
    return None