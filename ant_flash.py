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

# 打开参数文件并进行解析，传入一个文件路径，返回一个字典
def phase_config(file_path: str) -> dict:
    config = dict()

    try:
        f = open(file_path, 'r')
    except FileNotFoundError as e:
        print(e)
        print(f"Error: File {file_path} not found.")
        return config
    
    content = f.readlines()
    for line in content:
        # 跳过空行和注释行
        if not line or line.startswith('#'):
            continue
        line = line.strip()
        line = line.split('=', 1)
        var_name = line[0].strip()
        var_value = line[1].strip()
        # 解析变量值
        config[var_name] = phase_num_string(var_value)

    return config


def find_aimed_value(config: dict, var_name: str) -> Union[int, float, None]:
    var_value = config[var_name]
    if var_value == None:
        print("No find aimed key!")
    return var_value
    

# 调试程序
if __name__ == "__main__":
    test_strings = ["123", "45.67", "hello", "-89", "3.14159", "world123"]

    for s in test_strings:
        result = phase_num_string(s)
        print(f"Input: {s} => Output: {result} (Type: {type(result).__name__})")

    config = phase_config("config.txt")

    print("Parsed Successfully:")
    for key, value in config.items():
        print(f"{key} = {value} (Type: {type(value).__name__})")


