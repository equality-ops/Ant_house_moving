# Ant House Moving

第 21 届全国大学生智能车竞赛缩微组“蚂蚁搬家”相关代码。

by 21st 南工绝影一队

本代码采取全局惯性导航方案，通过三轮全向小车惯性导航，tof测距模块，yolo目标识别实现双车协同推动小球，沙袋，小熊等物体

主要硬件：
MCU：恩智浦rt1021（micropython）
IMU：逐飞科技imu660RB（意法半导体LSM6DSLTR六轴传感器）
tof：意法半导体VL53L4CD模块
## Directory Guide

| Directory | Purpose | Main entry |
| --- | --- | --- |
| `main_car/` | 主车：扫描、物体选择、路径规划、视觉伺服、协同搬运 | `main_car/user_main.py` |
| `slave_car/` | 从车：接收主车指令、惯导、伺服、环绕、TOF 距离控制 | `slave_car/user_main.py` |
| `openmv/` | OpenMV / OpenART 视觉侧程序 | device-specific |
| `assisting car/` | 辅助车相关程序和资料 | - |（已弃用）
| `assiting car camera/` | 辅助车摄像头相关程序和资料 | - |（已弃用）
| `tools/` | 上位机、路径与障碍调试工具 | `tools/boundary_debug_viewer.py` |
| `seekfree_demo/` | 逐飞底层示例 | - |
| `stubs/` | MicroPython 类型桩文件 | - |

## Main-Car Modules

| File | Responsibility |
| --- | --- |
| `ant_task.py` | 顶层任务状态机，组织扫描、选物体、导航、搬运、校准 |
| `ant_boundary_plan.py` | 九宫格物体选择、障碍膨胀、到边界的一次转向路径规划 |
| `ant_plan.py` | 惯导路径跟踪、转角控制、速度规划 |
| `ant_else.py` | 蜂鸣器、主从车无线通讯的发送与解析、车与openart摄像头的通讯、txt参数读取 |
| `ant_move.py` | 主从协同的 navigate / servo / orbit / move 状态机 |
| `ant_vision.py` | 控制视觉伺服，环绕与 AprilTag 校准（已弃用） |
| `ant_motor.py` | 底层编码器里程计、四元数姿态、角度环、三轮速度 PID |
| `main_config.txt` | 主车控制、速度、视觉、规划参数 |
| `user_main.py` | 硬件初始化、定时器回调与主程序入口 |

## Slave-Car Modules

从车的大部分控制结构与主车对应。额外的关键文件为：

| File | Responsibility |
| --- | --- |
| `vl53l4cd.py` | VL53L4CD TOF I2C 驱动 |
| `ant_move.py` | 额外有TOF控制类|
| `ant_move.py` | TOF 距离 PID、主车命令解析及协同搬运流程 |
| `slave_config.txt` | 从车独立的控制参数 |

## State Flow

主车状态机：


```text
READY_NAVIGATE -> MOVE（NAVIGATE / SCAN -> SERVO -> ORBIT -> MOVE）->READY_NAVIGATE
                                                                  /->RETURN
```
（ready_navigate后的状态机全被塞进了move中）
搬运过程中，主车负责选择目标、规划路径并发送消息；从车根据收到的目标点和方向进入惯导、伺服、环绕或协同推动状态。具体状态转换在 `main_car/ant_move.py` 和 `slave_car ant_move.py`。

## Navigation And Speed Planning

### Normal navigation

`NavigationPlan.navigate()` 使用“路径段 + 中继点速度 + 位置驱动 S 曲线”的规划方式：

1. 默认先转到 `target_turn_angle`，角度误差不大于约 `1.5 deg` 后开始平移；`if_first_turn=False` 时允许边转边走。
2. 当前车位置会自动作为路径第一个点。
3. 中继点速度按前后段夹角降低：转角越大，通过速度越低。
4. 每段根据 `acc_normal_coef`、`dec_coef`、`branch_threshold` 和 `final_threshold` 计算加速距离、减速距离与可达到的峰值速度。
5. 距离不足以完成“加速 - 巡航 - 减速”时，自动降低 `v_peak`，形成三角速度曲线。
6. 中间点在 `branch_threshold` 内切换下一段；最终点在 `final_threshold` 内停止。
7. 可通过if_high_angle,if_first_turn参数控制是否选择大角度旋转，是否边走边转

主要参数在 `main_car/main_config.txt`：

```ini
min_start_v = 40
long_v_max = 200
find_line_v_max = 80
acc_normal_coef = 0.03
dec_coef = 0.08
final_threshold = 1.0
branch_threshold = 3.0
```

### Push / MOVE

`MOVE` 状态的速度设计与普通导航不同：当前采用“推动最大速度 + 靠近目标边界限速”，不使用普通导航的分段 S 曲线。

- `move_v_max_T`、`move_v_max_S`、`move_v_max_B` 决定不同物体推动速度上限。
- 离对应边界小于 `20 cm` 时，目标速度由 `move_v_max` 按三次比例逐渐降到 `find_line_v_max`，为光电寻线和最终停车留出余量。
- `keep_x_or_y_v=True` 时按 `x` 到左右边界的距离判断减速；`False` 时按 `y` 到上下边界的距离判断减速。
- `fitting_path_` 非空时，原路径用于保持推动主方向，内收路径用于实际行驶航向与到点判断。

## Boundary Planning Cache

## TOF / I2C Notes

从车 TOF 的 I2C 访问必须在普通主循环中执行，不能放进电机、姿态或任务状态机定时器回调。控制定时器只读取缓存距离；这样单次 I2C 异常不会拖住四元数、编码器积分和速度环。

VL53L4CD 驱动采用有限重试、状态缓存和故障退避。若 SDA/SCL 被硬件永久拉低，软件重试无法恢复，应检查供电、上拉电阻、接线，并考虑使用 XSHUT 或 GPIO 时钟脉冲做总线恢复。

## Development Notes

- 目标硬件运行环境为 MicroPython；本机 CPython 只能做语法和纯算法检查。
- `main_config.txt` 与 `slave_config.txt` 是现场调参的主要入口。
- 编码器方向、IMU 安装方向、坐标系定义或电机 PID 修改后，应先单独验证底层闭环，再测试导航和主从协同。
- 不要在高频定时器回调中执行路径规划、文件写入、长时间串口输出或阻塞 I2C。
