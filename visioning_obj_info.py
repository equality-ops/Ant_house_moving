import matplotlib.pyplot as plt
import matplotlib.patches as patches
import ast

def visualize_map():
    # 解决 matplotlib 中文显示问题 (参考你提供的配置)
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Arial Unicode MS']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 开启 matplotlib 的交互模式
    plt.ion() 
    
    # 第一次创建画布和坐标轴
    fig, ax = plt.subplots(figsize=(8, 10))
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
        'S': '红沙袋(S)',
        'T': '网球(T)',
        'E': '蓝沙袋(E)',
        'W': '白熊(W)',
        'B': '棕熊(B)'
    }

    print("=== 地图可视化程序已更新 (增加搬运箭头与步骤) ===")
    print("列表中元素的排列顺序即为搬运顺序！")
    print("示例输入: [(160.0, 86.6, 'S', 'D'), (160.0, 153.3, 'W', 'U'), (39.0, 26.0, 'B')]")
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
                fig, ax = plt.subplots(figsize=(8, 10))
                plt.show(block=False)
            
            # 清空上一轮画面
            ax.clear()
            
            # 1. 设置地图基础属性 (320x240)
            ax.set_xlim(0, 320.0)
            ax.set_ylim(0, 240.0)
            ax.set_title("地图搬运路径可视化 (320x240)", fontsize=16)
            ax.set_xlabel("X 坐标")
            ax.set_ylabel("Y 坐标")
            ax.set_facecolor('#f0f0f0') 
            
            # 2. 绘制中心 100 * 100 矩形框
            center_box = patches.Rectangle(
                (110.0, 70.0), 100.0, 100.0, 
                linewidth=1.5, edgecolor='red', facecolor='none', 
                linestyle='--', zorder=1
            )
            ax.add_patch(center_box)
            
            # 绘制上下推落点参考线
            ax.axhline(y=0.0, color='red', linestyle='-', linewidth=2, label="下边界推落点 (DOWN)", alpha=0.5)
            ax.axhline(y=240.0, color='green', linestyle='-', linewidth=2, label="上边界推落点 (UP)", alpha=0.5)
            
            # 3. 使用 enumerate 获取搬运顺序 step_idx
            for step_idx, item in enumerate(items):
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
                
                marker_shape = 'o' 
                
                # 如果有方向信息，绘制箭头并调整形状
                if direction:
                    direction = direction.upper()
                    
                    # 确定箭头终点 y 坐标
                    end_x = x
                    end_y = 240.0 if direction == 'U' else 0.0
                    
                    if direction == 'U':
                        marker_shape = '^'
                        label_name += ' [↑]'
                    elif direction == 'D':
                        marker_shape = 'v'
                        label_name += ' [↓]'

                    # 绘制搬运箭头
                    ax.annotate('', xy=(end_x, end_y), xytext=(x, y),
                                arrowprops=dict(facecolor='blue', edgecolor='blue', alpha=0.5,
                                                width=2, headwidth=8, shrink=0.05),
                                zorder=2)
                    
                    # 在箭头中点标记搬运顺序
                    mid_x = x + 4
                    mid_y = (y + end_y) / 2
                    ax.text(mid_x, mid_y, f"步骤 {step_idx + 1}",
                            color='darkred', fontsize=11, fontweight='bold',
                            bbox=dict(facecolor='white', edgecolor='none', alpha=0.8),
                            zorder=4)

                # 绘制物体点
                ax.scatter(x, y, c=point_color, s=150, edgecolors='black', 
                           marker=marker_shape, zorder=5)
                
                # 添加物体文本标签 (包含步骤序号)
                ax.text(x + 4, y + 4, f"{label_name}\n#{step_idx+1}", 
                        fontsize=10, color='black', fontweight='bold',
                        bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', pad=1.5),
                        zorder=6)

            # 4. 图表收尾设置
            ax.legend(loc="upper left")
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