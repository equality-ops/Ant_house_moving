import matplotlib.pyplot as plt
import matplotlib.patches as patches
import ast

def visualize_map():
    # 开启 matplotlib 的交互模式
    plt.ion() 
    
    # 第一次创建画布和坐标轴
    fig, ax = plt.subplots(figsize=(8, 6))
    plt.show(block=False)
    
    # 定义物品代号对应的颜色
    color_map = {
        'S': 'red',          # 红沙袋
        'T': 'lawngreen',    # 网球
        'E': 'blue',         # 蓝沙袋
        'W': 'white',        # 白熊
        'B': 'saddlebrown'   # 棕熊
    }
    
    # 定义物品代号对应的中文名称
    name_map = {
        'S': '红沙袋 (S)',
        'T': '网球 (T)',
        'E': '蓝沙袋 (E)',
        'W': '白熊 (W)',
        'B': '棕熊 (B)'
    }

    print("=== 地图可视化程序已更新 (支持搬运方向) ===")
    print("支持三种元素的普通格式: (x, y, '种类')")
    print("支持四种元素的方向格式: (x, y, '种类', '方向')  *方向为 'U'(上) 或 'D'(下)")
    print("示例输入: [(160.0, 86.6, 'S', 'D'), (39.0, 26.0, 'B')]")
    print("输入 'q' 或 'quit' 退出程序\n")

    while True:
        # 接收用户输入
        user_input = input("等待输入新数据: ")
        
        # 判断是否退出
        if user_input.strip().lower() in ['q', 'quit']:
            print("程序已退出。")
            break
            
        try:
            items = ast.literal_eval(user_input)
            
            # 检测窗口是否被手动关闭
            if not plt.fignum_exists(fig.number):
                fig, ax = plt.subplots(figsize=(8, 6))
                plt.show(block=False)
            
            # 清空上一轮画面
            ax.clear()
            
            # 1. 设置地图基础属性
            ax.set_xlim(0, 320.0)
            ax.set_ylim(0, 240.0)
            ax.set_title("Map Visualization (320x240) with Directions")
            ax.set_xlabel("X Axis")
            ax.set_ylabel("Y Axis")
            ax.set_facecolor('#f0f0f0') 
            
            # 2. 绘制中心 100 * 100 矩形框
            center_box = patches.Rectangle(
                (110.0, 70.0), 100.0, 100.0, 
                linewidth=1.5, edgecolor='red', facecolor='none', 
                linestyle='--', zorder=1
            )
            ax.add_patch(center_box)
            
            # 3. 遍历列表并在地图上绘制物体
            for item in items:
                # 兼容性判断：检查是 3 个元素还是 4 个元素
                direction = None
                if len(item) == 3:
                    x, y, obj_code = item
                elif len(item) == 4:
                    x, y, obj_code, direction = item
                else:
                    print(f"[警告] 跳过格式不正确的数据: {item}")
                    continue
                    
                obj_code = obj_code.upper()
                point_color = color_map.get(obj_code, 'gray')
                label_name = name_map.get(obj_code, f'未知({obj_code})')
                
                # 默认形状为圆形 'o'
                marker_shape = 'o' 
                
                # 如果有方向信息，修改形状和文字标签
                if direction:
                    direction = direction.upper()
                    if direction == 'U':
                        marker_shape = '^'  # 向上三角形
                        label_name += ' [↑向上]'
                    elif direction == 'D':
                        marker_shape = 'v'  # 向下三角形
                        label_name += ' [↓向下]'
                
                # 绘制物体点 (应用动态决定的 marker_shape)
                ax.scatter(x, y, c=point_color, s=120, edgecolors='black', 
                           marker=marker_shape, zorder=3)
                
                # 添加文本标签
                ax.text(x + 4, y + 4, label_name, fontsize=9, color='black', 
                        bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', pad=1.5),
                        zorder=4)

            ax.grid(True, linestyle=':', alpha=0.7, zorder=0)
            
            # 刷新画布并将窗口顶层显示
            plt.draw()
            plt.pause(0.1) 
            
        except SyntaxError:
            print("[错误] 输入格式有误，请确保输入的是 Python 列表格式。\n")
        except ValueError:
            print("[错误] 无法解析输入，请检查数据结构。\n")
        except Exception as e:
            print(f"[错误] 发生未知错误: {e}\n")

    # 退出前清理资源
    plt.ioff()
    plt.close('all')

if __name__ == "__main__":
    visualize_map()