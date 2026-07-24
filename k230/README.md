# K230两阶段房号视觉程序

该程序保留现有YOLOv8数字模型，把视觉流程改成与参考OpenMV工程相同的两阶段思路：

```text
TASK1：初次确认目标房号
        ↓ MSPM0记录房号并发送TASK2命令
TASK2：K230只寻找该目标数字
        ↓ 返回数字在左/中/右
MSPM0根据路线观察点决定最终房间和转向
```

K230只负责识别数字和左右位置，不直接控制电机。最终路线映射应放在MSPM0的
`main.c`中。

## 文件

- `main.py`：摄像头、YOLOv8推理、两阶段任务、LCD和双向UART通信。
- `best.kmodel`：与代码标签和输入尺寸匹配的模型。
- `uart_test.py`：不加载模型，只测试K230与MSPM0双向通信。
- `uart_tx_test.py`：不等待MSPM0命令，每200ms固定发送一次1号房TASK1结果，单独测试K230到MSPM0方向。

## 接线

| K230庐山派 | MSPM0G3507 | 数据方向 |
|---|---|---|
| GPIO11 / UART2_TX | PB16 / UART2_RX | K230识别结果到MSPM0 |
| GPIO12 / UART2_RX | PB15 / UART2_TX | MSPM0任务命令到K230 |
| GND | GND | 必须共地 |

当前工程使用原来的`PB16`作为UART2接收引脚；`PB15`作为UART2发送引脚。

`target_room`只从第二行这条反向串口线接收。若漏接`PB15 -> GPIO12`，K230仍可把
识别结果发给MSPM0，但`target_room`会一直为默认值`0`。

上表的GPIO11/12是UART2专用座（丝印`T`/`R`）使用的逻辑GPIO编号，不是40Pin
排针的物理针脚号。若使用40Pin排针，应接物理`Pin11=GPIO5/TX`、
`Pin13=GPIO6/RX`，并把Python脚本中的`UART_USE_40PIN_HEADER`设为`True`。

双方都使用`115200、8N1`。连接前确认IO电平兼容，禁止把5V UART电平直接接入K230。

## TASK1：初次识别房号

MSPM0发送：

```c
k230_set_task(K230_TASK_INITIAL_ROOM, 0);
```

对应串口命令：

```text
A5 01 00 5A
```

K230仍对整幅画面执行YOLO，但只接受检测框中心落在中央区域的房号1~8。这相当于
原OpenMV程序第一次只使用中央ROI，能够减少画面两侧其他数字干扰。中央区域宽度由：

```python
TASK1_CENTER_HALF_RATIO = 0.20
```

控制。`0.20`表示中线左右各20%，即中央40%画面。范围太窄就增大，误选两侧数字
就减小。TASK1结果的方向固定为`CENTER`。

## TASK2：只寻找目标数字并判断左右

假设TASK1识别到6号房，MSPM0发送：

```c
k230_set_task(K230_TASK_FIND_TARGET, 6);
```

对应串口命令：

```text
A5 02 06 5A
```

从此K230只在YOLO结果中选择数字6，画面中的其他数字全部忽略。然后用数字6检测框
的中心横坐标与画面中线比较：

```text
框中心 < 中线 - 死区：LEFT
框中心 > 中线 + 死区：RIGHT
中线附近              ：CENTER
```

死区由`SIDE_DEADBAND_RATIO = 0.08`控制。左右判断跳变时增大，中央区域太宽时减小。

## 串口协议

### MSPM0到K230命令

固定4字节：

```text
0xA5  TASK  TARGET  0x5A
```

- `TASK=1`：初次识别，`TARGET=0`。
- `TASK=2`：寻找目标，`TARGET=1~8`。

### K230到MSPM0结果

固定6字节：

```text
0xAA  ROOM  SIDE  FOUND  TASK  0x55
```

- `ROOM`：无结果时为0，识别成功时为1~8。
- `SIDE`：`0=CENTER、1=LEFT、2=RIGHT`。
- `FOUND`：`0=尚无稳定结果、1=连续多帧稳定识别成功`。
- `TASK=1/2`：该结果来自TASK1或TASK2。

例如TASK2识别到6号房在右侧：

```text
AA 06 02 01 02 55
```

## MSPM0调用示例

```c
k230_result_t vision;
uint8_t target_room = 0;

/* 上电后要求K230执行初次识别。 */
k230_set_task(K230_TASK_INITIAL_ROOM, 0);

while (1)
{
    if (k230_get_result(&vision))
    {
        if (vision.task == K230_TASK_INITIAL_ROOM && vision.found)
        {
            target_room = vision.room;

            /* 切换TASK2，后续只寻找初次确认的目标数字。 */
            k230_set_task(K230_TASK_FIND_TARGET, target_room);
        }
        else if (vision.task == K230_TASK_FIND_TARGET && vision.found)
        {
            if (vision.room == target_room &&
                vision.side == K230_SIDE_LEFT)
            {
                /* 目标数字在当前观察点左侧：在这里推进你的左侧路线状态。 */
            }
            else if (vision.room == target_room &&
                     vision.side == K230_SIDE_RIGHT)
            {
                /* 目标数字在当前观察点右侧：在这里推进你的右侧路线状态。 */
            }
        }
    }
}
```

不要在每次主循环都重复发送`k230_set_task()`。只在上电或任务/目标发生变化时发送，
K230会记住当前任务并持续执行。

## 识别消抖

`main.py`要求相同的`房号+方向`连续出现5帧才设置`FOUND=1`：

```python
UART_STABLE_FRAMES = 5
```

连续丢失5帧后清除旧结果：

```python
UART_CLEAR_FRAMES = 5
```

结果每200ms重发一次，MSPM0偶尔漏掉一帧也能收到后续结果。

## 推荐测试顺序

1. 在K230运行`uart_tx_test.py`，确认MSPM0能持续收到固定的1号房结果。
2. 在K230运行`uart_test.py`，再测试双向UART。
3. MSPM0发送TASK1，确认收到模拟的1号房中央结果。
4. MSPM0发送TASK2和目标1~8，确认收到对应目标的LEFT/RIGHT模拟结果。
5. 部署`main.py`和`best.kmodel`，只测试TASK1。
6. TASK1稳定后再测试TASK2，将目标数字分别放在画面左侧和右侧。
7. 视觉与通信稳定后，最后接入小车路线状态机。

## 部署路径

```text
/sdcard/apps/k230_digit/main.py
/sdcard/apps/k230_digit/best.kmodel
```

在CanMV IDE中验证通过后，如需上电自启动，再把主脚本保存为`/sdcard/main.py`。
