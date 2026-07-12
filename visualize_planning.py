import matplotlib.pyplot as plt
import matplotlib.patches as patches
from path_planner import PathPlanner


def visualize_plan(plan, status, planner: PathPlanner):
    """
    可视化九宫格路径规划结果

    参数:
        plan:   [(center_x, center_y, kind, direction), ...]  按搬运顺序排列
        status: 状态标志位
        planner: PathPlanner 实例（用于获取场地边界参数）
    """
    # 解决 matplotlib 中文显示问题
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Arial Unicode MS']
    plt.rcParams['axes.unicode_minus'] = False

    fig, ax = plt.subplots(figsize=(8, 10))
    box_left, box_bottom = planner.box_left, planner.box_bottom
    box_right, box_top = planner.box_right, planner.box_top
    push_down_y = planner.push_down_y
    push_up_y = planner.push_up_y

    # 1. 绘制场地外框
    rect = patches.Rectangle((box_left, box_bottom), box_right - box_left, box_top - box_bottom,
                             linewidth=2, edgecolor='black', facecolor='none', zorder=3)
    ax.add_patch(rect)

    # 2. 绘制九宫格辅助线
    cell_w = (box_right - box_left) / 3
    cell_h = (box_top - box_bottom) / 3
    for i in range(1, 3):
        ax.axvline(x=box_left + i * cell_w, color='gray', linestyle='--', alpha=0.5)
        ax.axhline(y=box_bottom + i * cell_h, color='gray', linestyle='--', alpha=0.5)

    # 3. 绘制物体（在格子中心）与搬运路径箭头
    colors = ['#FF9999', '#99CCFF', '#99FF99', '#FFCC99', '#E5CCFF',
              '#FFB3B3', '#B3D9FF', '#B3FFB3', '#FFE0B3', '#E8CCFF']

    for step_idx, (cx, cy, kind, direction) in enumerate(plan):
        color = colors[step_idx % len(colors)]

        # 画物体在格子中心的位置
        ax.scatter(cx, cy, c=color, s=300, edgecolors='black', zorder=4)
        ax.text(cx, cy, f"{kind}\n#{step_idx+1}",
                fontsize=10, ha='center', va='center', zorder=5, fontweight='bold')

        # 画搬运箭头：从格子中心 → 上/下边界
        end_x = cx
        end_y = push_up_y if direction == 'U' else push_down_y

        ax.annotate('', xy=(end_x, end_y), xytext=(cx, cy),
                    arrowprops=dict(facecolor='blue', edgecolor='blue', alpha=0.6,
                                    width=2, headwidth=8, shrink=0.05))

        # 在箭头中点标记搬运顺序
        mid_x = cx + 5
        mid_y = (cy + end_y) / 2
        ax.text(mid_x, mid_y, f"步骤 {step_idx + 1}",
                color='darkred', fontsize=12, fontweight='bold',
                bbox=dict(facecolor='white', edgecolor='none', alpha=0.7))

    # 4. 设置图表属性
    ax.set_xlim(box_left - 30, box_right + 30)
    ax.set_ylim(push_down_y - 20, push_up_y + 20)

    # 标出下边界和上边界
    ax.axhline(y=push_down_y, color='red', linestyle='-', linewidth=2, label="下边界推落点 (DOWN)")
    ax.axhline(y=push_up_y, color='green', linestyle='-', linewidth=2, label="上边界推落点 (UP)")

    # 标题包含状态信息
    status_names = {0: "正常", 1: "越界归类", 2: "冲突消解", 3: "满格终止"}
    ax.set_title(f"九宫格搬运路径规划可视化 [status={status} {status_names.get(status, '?')}]", fontsize=16)
    ax.set_xlabel("X 坐标 (cm)")
    ax.set_ylabel("Y 坐标 (cm)")
    ax.legend(loc="upper left")
    ax.grid(True, linestyle=':', alpha=0.6)

    plt.show()


if __name__ == "__main__":
    planner = PathPlanner()
    test_objects = [(132.9738182990603, 154.4226813980545, 'E'), (162.794408029426, 117.8407909215069, 'W'), (195.3693425205868, 123.6875655427025, 'W'), (167.1514072445506, 92.8249841553928, 'T'), (194.6978377420619, 164.3810868510359, 'T'), (133.6805132106046, 120.5940828862139, 'B'), (137.0943255836747, 86.91661617580959, 'S'), (145.8148561363436, 115.3154870504014, 'B'), (196.7971969038164, 88.79980893609034, 'B')]

    plan, status = planner.plan_path(test_objects)

    # 运行可视化
    visualize_plan(plan, status, planner)
