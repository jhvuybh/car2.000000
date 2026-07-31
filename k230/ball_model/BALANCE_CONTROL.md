# 单滑轨平衡钢球控制方案

## 控制链路

```text
K230摄像头 -> YOLO钢球位置P(0~1000) -> UART -> MSPM0
                                              |
                                              v
                                    位置/速度PD控制器
                                              |
                                              v
                                   DIR + STEP脉冲频率
                                              |
                                              v
                              闭环步进电机 -> 滑轨 -> 水管倾角
```

摄像头固定后，控制量应使用钢球沿水管方向的位置。水管在画面中左右延伸时，
`TRACK_AXIS=0`；上下延伸时使用 `TRACK_AXIS=1`。

## K230到MSPM0协议

`ball_balance_vision.py` 每次识别发送8字节二进制帧：

```text
AA SEQ VALID POS_H POS_L CONF CHECKSUM 55
```

- `SEQ`：帧序号，0~255循环。
- `VALID`：1表示识别到钢球，0表示丢球。
- `POS`：沿水管方向归一化位置0~1000；丢球时为`0xFFFF`。
- `CONF`：置信度0~100。
- `CHECKSUM`：`SEQ`到`CONF`五个字节之和的低8位。

接线：K230 GPIO11/UART2_TX 接 MSPM0 PB16/UART2_RX，两块板必须共地。

## 推荐控制算法

控制周期建议10 ms。相机每来一帧，先对位置做低通滤波并计算球速：

```c
filtered_position += 0.35f * (position - filtered_position);
ball_velocity = (filtered_position - previous_position) / vision_dt;
error = target_position - filtered_position;
tube_height_offset = kp * error - kd * ball_velocity;
target_steps = level_steps + tube_height_offset;
```

这里的PD输出作为相对“水平位置”的滑轨目标位置，而不是直接作为无限制的电机
速度。MSPM0再用带速度、加速度限制的脉冲发生器追踪`target_steps`。目标位置应
限制在滑轨允许的范围，并加入：

1. 上、下机械限位开关；
2. 滑轨软件位置上下限；
3. 加速度限制，避免水管猛跳；
4. 超过200 ms没有有效视觉帧时停止脉冲；
5. 丢球时先让水管缓慢回到水平位置，不要继续积分；
6. 上电先低速回零，再建立滑轨零点。

初次调试只使用P控制，从很小的`kp`开始。确认钢球偏左时水管倾斜方向能够
让它向右滚，再逐步增加`kp`；开始来回振荡后加入`kd`抑制速度。建议先不开
积分项，钢球系统延迟较大，积分很容易造成持续振荡。

## 还需要确认的电机参数

张大头闭环步进存在X42、X57、Emm42、X42S等不同版本，串口指令和接口可能
不同。完成MSPM0电机代码前需要确认：

- 电机/驱动器完整型号和版本；
- 使用`STEP/DIR`还是驱动器串口；
- 细分数、丝杆导程、允许行程；
- 上下限位开关接到MSPM0的引脚；
- STEP、DIR、EN计划使用的MSPM0引脚。
