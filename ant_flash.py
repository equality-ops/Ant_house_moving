from typing import Union

# 将字符串解析为整数或浮点数，如果无法解析则返回原始字符串
def phase_num_string(s: str) -> Union[int, float, str]:

    # 尝试解析为整数(只支持十进制)
    try:
        value = int(s, 10)
        return value
    except ValueError:
        pass

    # 尝试解析为浮点数
    try:
        value = float(s)
        return value
    except ValueError:
        pass

    # 如果无法解析为数字，则返回原始字符串
    return s


def phase_config(file_path: str) -> dict:
    config = dict()
    with open(file_path, 'r') as f:
        content = f.readlines()
        for line in content:
            if not line or line.startswith('#'):
                continue
            line = line.strip()
            line = line.split('=', 1)
            var_name = line[0].strip()
            var_value = line[1].strip()
            config[var_name] = phase_num_string(var_value)

    return config

# 调试程序
if __name__ == "__main__":
    test_strings = ["123", "45.67", "hello", "-89", "3.14159", "world123"]

    for s in test_strings:
        result = phase_num_string(s)
        print(f"Input: {s} => Output: {result} (Type: {type(result).__name__})")