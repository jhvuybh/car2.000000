"""
庐山派 K230：YOLOv8 房号识别并通过 UART2 发送给 MSPM0G3507。

程序执行流程：
    接收MSPM0发来的视觉任务命令
        -> TASK1：初次识别画面中央的房号
        -> TASK2：只寻找MSPM0指定的目标房号
    摄像头取图
        -> AI2D 按 320x320 做 letterbox 预处理
        -> YOLOv8 推理及 NMS 后处理
        -> 根据当前任务筛选检测框
        -> TASK2根据检测框中心横坐标判断左/中/右
        -> 连续多帧结果一致后确认
        -> LCD 显示检测框和状态
        -> UART2 向MSPM0发送房号、方向、成功标志和当前任务

板端文件：
    /sdcard/apps/k230_digit/main.py
    /sdcard/apps/k230_digit/best.kmodel

接线：
    K230 GPIO11 (UART2_TX) -> MSPM0 PB16 (K230_UART2_RX)
    K230 GPIO12 (UART2_RX) <- MSPM0 PB15 (K230_UART2_TX)
    K230 GND               -> MSPM0 GND

注意：target_room由MSPM0通过第二条线发给K230；如果只接GPIO11和GND，
K230仍能发送识别结果，但target_room会一直保持默认值0。

MSPM0 -> K230任务命令（4字节二进制帧）：
    A5 TASK TARGET 5A
    TASK=1：初次识别，TARGET应为0
    TASK=2：定向寻找，TARGET为要寻找的房号1~8

K230 -> MSPM0识别结果（6字节二进制帧）：
    AA ROOM SIDE FOUND TASK 55
    ROOM ：0表示无结果，1~8表示房号
    SIDE ：0=中间，1=左侧，2=右侧
    FOUND：0=尚未稳定识别，1=稳定识别成功
    TASK ：0=K230已就绪；1/2=产生该结果时正在执行的视觉任务

以上字段都是二进制数值，不是ASCII字符。例如TASK2识别到3号房在左侧：
    AA 03 01 01 02 55
"""

# CanMV 固件自带的摄像头/显示管线，负责取帧、LCD显示和媒体资源管理。
from libs.PipeLine import PipeLine
# CanMV 封装的 YOLOv8 推理类，包含模型加载、推理、后处理和检测框绘制。
from libs.YOLO import YOLOv8
# ScopedTiming 用于可选耗时统计；letterbox_pad_param 计算等比例缩放和补边参数。
from libs.Utils import ScopedTiming, letterbox_pad_param
# FPIOA 用于把芯片内部 UART2 功能映射到实际 GPIO；UART 用于串口通信。
from machine import FPIOA, UART
# nncase_runtime 提供 K230 KPU/AI2D 的运行时枚举和接口。
import nncase_runtime as nn
# K230 内存有限，循环中主动垃圾回收可以降低长时间运行时内存碎片风险。
import gc
import os
import sys
import time


# ---------------------------- 用户配置区 ----------------------------
# kmodel 必须提前复制到 K230 SD 卡的这个路径。
KMODEL_PATH = "/sdcard/apps/k230_digit/best.kmodel"

# 必须和训练 data.yaml 的 names 顺序一致：类别 0~8 对应数字 1~9。
# YOLO输出的是类别索引，例如 class_id=0；程序再通过 LABELS 将它转换为真实数字1。
LABELS = ["1", "2", "3", "4", "5", "6", "7", "8", "9"]
# 小车只有1~8号目标，因此9虽然可以在LCD上被模型检测出来，但不会成为有效发送结果。
VALID_ROOMS = (1, 2, 3, 4, 5, 6, 7, 8)
# 必须与导出/转换 kmodel 时使用的 imgsz 一致。
MODEL_INPUT_SIZE = [320, 320]

# 庐山派板载 ST7701 LCD。
# DISPLAY_SIZE=None 表示让 PipeLine 从固件读取屏幕的实际分辨率。
DISPLAY_MODE = "lcd"
DISPLAY_SIZE = None
# 摄像头提供给AI的RGB888平面格式分辨率：[宽, 高]。
# 分辨率越大，画面细节越多，但取帧、缩放和显示的内存开销也越大。
RGB888P_SIZE = [640, 360]

# 置信度低于0.50的框会在YOLO后处理中被丢弃。
CONFIDENCE_THRESHOLD = 0.50
# NMS用于删除同一目标的重叠框；数值越小，抑制重叠框越强。
NMS_THRESHOLD = 0.45
# 单帧最多保留的检测框数量，避免异常画面产生过多框占用内存和时间。
MAX_BOXES_NUM = 20

# 庐山派 UART2 引脚，115200 8N1。
# False：使用板载UART2专用座/大焊盘的T、R，对应逻辑GPIO11、GPIO12。
# True ：使用40Pin排针的物理Pin11、Pin13，对应逻辑GPIO5、GPIO6。
UART_USE_40PIN_HEADER = False
if UART_USE_40PIN_HEADER:
    UART_TX_PIN = 5
    UART_RX_PIN = 6
else:
    UART_TX_PIN = 11
    UART_RX_PIN = 12
UART_BAUDRATE = 115200

# 连续识别到完全相同的“房号+方向”5帧后才确认，避免单帧误识别。
UART_STABLE_FRAMES = 5
# 连续5帧没有有效房号后清除旧结果；短暂遮挡不会立即清除。
UART_CLEAR_FRAMES = 5
# 结果不变时每200ms重发一次，使MSPM0偶尔漏掉一帧时仍能再次收到。
UART_HEARTBEAT_MS = 200
# 单次UART.write()可能只接收部分缓冲区；在此时间内继续补写剩余字节。
UART_WRITE_TIMEOUT_MS = 50
# PB16接收端忙于其他中断时，连续6字节可能挤满硬件FIFO；逐字节留出搬运时间。
UART_INTER_BYTE_MS = 1

# K230上电后即使尚未收到MSPM0命令，也默认进入TASK1初次房号识别。
DEFAULT_TASK = 1
DEFAULT_TARGET_ROOM = 0

# TASK1只接受检测框中心落在画面中央区域内的数字。
# 0.20表示以画面中线为中心，左右各允许20%画面宽度，即使用中央40%区域。
# 如果初次识别范围太窄可增大；如果经常误选两侧其他数字可减小。
TASK1_CENTER_HALF_RATIO = 0.20

# 检测框中心与画面中线的距离小于画面宽度的 8% 时，判定为中间。
# 如果左右判断反复跳变，可增大；如果中间区域太宽，可减小。
SIDE_DEADBAND_RATIO = 0.08
# ------------------------------------------------------------------

# 两种视觉任务编号，必须与MSPM0端k230_task_t保持一致。
TASK_INITIAL_ROOM = 1
TASK_FIND_TARGET = 2
TASK_NAMES = ("UNUSED", "INITIAL", "FIND_TARGET")

# SIDE数值会原样放入UART帧的第3字节，必须与MSPM0端k230_side_t保持一致。
SIDE_CENTER = 0
SIDE_LEFT = 1
SIDE_RIGHT = 2
# 仅用于LCD/串口调试打印；索引0、1、2分别对应上面的方向值。
SIDE_NAMES = ("CENTER", "LEFT", "RIGHT")

# MSPM0 -> K230命令帧边界。
COMMAND_FRAME_HEADER = 0xA5
COMMAND_FRAME_TAIL = 0x5A

# K230 -> MSPM0结果帧边界。
RESULT_FRAME_HEADER = 0xAA
RESULT_FRAME_TAIL = 0x55


class DigitYOLOv8(YOLOv8):
    """定制YOLOv8预处理，使板端预处理与训练/量化校准保持一致。

    原始画面是640x360，模型输入是320x320。直接拉伸到正方形会使数字变形，
    因此先保持宽高比缩放，再用像素值114补齐空白区域（letterbox）。
    """

    def config_preprocess(self, input_image_size=None):
        # debug_mode>0时ScopedTiming会打印本段配置耗时；正常运行debug_mode=0不打印。
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            # 未显式指定输入尺寸时，使用YOLO对象保存的摄像头AI通道尺寸。
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size

            # 计算上、下、左、右需要补多少像素，并保存缩放比例到self.scale，
            # 后处理会依靠相同的缩放关系把检测框恢复到显示画面坐标。
            top, bottom, left, right, self.scale = letterbox_pad_param(
                self.rgb888p_size, self.model_input_size
            )

            # pad参数按NCHW四维张量描述。补边颜色[114,114,114]与Ultralytics默认一致。
            self.ai2d.pad(
                [0, 0, 0, 0, top, bottom, left, right],
                0,
                [114, 114, 114],
            )

            # 使用双线性插值缩放图像；half_pixel模式需与模型转换时的设置一致。
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)

            # 构建AI2D硬件预处理管线。形状均为[N, C, H, W]，即NCHW格式：
            # 输入 [1,3,360,640] -> 输出 [1,3,320,320]。
            self.ai2d.build(
                [1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                [1, 3, self.model_input_size[1], self.model_input_size[0]],
            )


def file_exists(path):
    """检查SD卡文件是否存在，避免模型路径错误时直接抛出难理解的加载异常。"""
    try:
        # CanMV MicroPython中os.stat成功表示路径存在。
        os.stat(path)
        return True
    except OSError:
        return False


def init_uart():
    """配置K230引脚复用并创建UART2对象，返回给主程序使用。"""
    # K230的外设功能需要先通过FPIOA绑定到板上实际GPIO。
    fpioa = FPIOA()
    # GPIO11作为K230发送脚，接MSPM0 PB16/UART2_RX。
    fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
    # GPIO12作为K230接收脚，接MSPM0 PB15/UART2_TX；target_room从这条线接收。
    fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)

    # 双方必须设置为相同的115200波特率、8数据位、无校验、1停止位（8N1）。
    uart = UART(
        UART.UART2,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )
    print(
        "UART2: TX=GPIO%d RX=GPIO%d, %d 8N1"
        % (UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE)
    )
    return uart


class CommandParser:
    """解析MSPM0发来的4字节任务命令：A5 TASK TARGET 5A。

    UART是连续字节流，一次uart.read()可能只读到半帧，也可能同时读到多帧。
    因此使用状态机逐字节解析，并把状态保存在对象中留给下一次poll()继续使用。

    状态含义：
        0：等待帧头0xA5
        1：等待任务号1或2
        2：等待目标房号0~8
        3：等待帧尾0x5A
    """

    def __init__(self):
        self.state = 0
        self.task = DEFAULT_TASK
        self.target_room = DEFAULT_TARGET_ROOM

    def feed_byte(self, value):
        """输入一个UART字节；收到完整有效命令时返回(task, target)，否则返回None。"""
        if self.state == 0:
            # 丢弃帧头之前的噪声字节。
            if value == COMMAND_FRAME_HEADER:
                self.state = 1

        elif self.state == 1:
            # 任务号只允许TASK1或TASK2。
            if value == TASK_INITIAL_ROOM or value == TASK_FIND_TARGET:
                self.task = value
                self.state = 2
            else:
                # 当前字节若又是0xA5，则将它直接作为新帧头，快速重新同步。
                self.state = 1 if value == COMMAND_FRAME_HEADER else 0

        elif self.state == 2:
            # TASK1目标为0；TASK2目标为1~8。这里先接受0~8，帧尾处再结合任务校验。
            if value <= 8:
                self.target_room = value
                self.state = 3
            else:
                self.state = 1 if value == COMMAND_FRAME_HEADER else 0

        elif self.state == 3:
            # 先保存收到帧尾后的下一状态，错误帧尾恰好是新帧头时不会丢失同步。
            next_state = 1 if value == COMMAND_FRAME_HEADER else 0
            command_valid = (
                value == COMMAND_FRAME_TAIL
                and (
                    self.task == TASK_INITIAL_ROOM
                    or (
                        self.task == TASK_FIND_TARGET
                        and self.target_room in VALID_ROOMS
                    )
                )
            )
            self.state = next_state

            if command_valid:
                # TASK1不需要目标房号，统一强制为0，避免旧TARGET引起误解。
                if self.task == TASK_INITIAL_ROOM:
                    self.target_room = 0
                return self.task, self.target_room

        else:
            # 状态值异常时恢复到等待帧头。
            self.state = 0

        return None

    def poll(self, uart):
        """非阻塞读取当前UART缓存；返回最新完整命令，没有新命令则返回None。"""
        data = uart.read()
        if data is None:
            return None

        newest_command = None
        for value in data:
            command = self.feed_byte(value)
            if command is not None:
                # 一次读取到多帧命令时采用最后一帧，使最新任务立即生效。
                newest_command = command
        return newest_command


def select_vision_result(result, task, target_room, frame_width):
    """根据当前任务筛选YOLO检测结果。

    CanMV YOLO检测后处理result结构：
        result[0]：检测框列表，每个框为[x, y, width, height]
        result[1]：每个框对应的类别索引class_id
        result[2]：每个框对应的置信度score

    TASK1（初次识别）：
        只接受检测框中心位于画面中央区域的房号1~8；方向固定返回CENTER。
        这对应原OpenMV程序“第一次只看中间ROI”的设计。

    TASK2（定向寻找）：
        只接受类别等于target_room的检测框，然后根据框中心判断左/中/右。
        即使画面中还有其他数字，也不会干扰目标数字的路线判断。

    返回：
        (room, side, score)
        没有有效结果时返回(None, None, 0.0)。
    """
    # 同时检查None、结果组成数量以及检测框数量，避免访问空数组。
    if not result or len(result) < 3 or len(result[0]) == 0:
        return None, None, 0.0

    # 初值表示当前还没有找到1~8范围内的有效检测框。
    best_room = None
    best_side = None
    best_score = 0.0

    # 一帧可能同时出现多个数字，遍历全部候选框，最终保留符合当前任务且置信度最高的框。
    for index in range(len(result[2])):
        class_id = int(result[1][index])
        # 防止模型输出异常类别索引导致LABELS数组越界。
        if class_id < 0 or class_id >= len(LABELS):
            continue

        # 将类别索引映射为真实房号。例如class_id=2 -> LABELS[2]="3" -> room=3。
        room = int(LABELS[class_id])
        score = float(result[2][index])

        # 房号9不属于小车有效目标，直接跳过。
        if room not in VALID_ROOMS:
            continue

        # CanMV YOLO后处理框格式为[x, y, width, height]。
        box = result[0][index]
        # x是检测框左边缘坐标，所以中心横坐标=x+width/2。
        box_center_x = float(box[0]) + float(box[2]) * 0.5
        # frame_width使用LCD显示坐标宽度，因为YOLO后处理框已映射到显示坐标。
        image_center_x = frame_width * 0.5

        if task == TASK_INITIAL_ROOM:
            # TASK1只选画面中央区域的数字，模拟原OpenMV第一次识别使用中央ROI。
            task1_half_width = frame_width * TASK1_CENTER_HALF_RATIO
            if abs(box_center_x - image_center_x) > task1_half_width:
                continue
            side = SIDE_CENTER

        elif task == TASK_FIND_TARGET:
            # TASK2只寻找MSPM0指定的目标房号，其他数字全部忽略。
            if room != target_room:
                continue

            # 中心死区左右各占画面宽度的8%。以800像素屏幕为例，deadband=64像素。
            deadband = frame_width * SIDE_DEADBAND_RATIO
            if box_center_x < image_center_x - deadband:
                side = SIDE_LEFT
            elif box_center_x > image_center_x + deadband:
                side = SIDE_RIGHT
            else:
                side = SIDE_CENTER

        else:
            # 未定义任务不产生视觉结果。
            continue

        # 当前框满足任务要求时，和之前候选比较，只保留置信度更高者。
        if score > best_score:
            best_room = room
            best_side = side
            best_score = score

    return best_room, best_side, best_score


def send_result(uart, room, side, found, task):
    """发送固定6字节结果帧：[0xAA, 房号, 方向, 成功标志, 任务, 0x55]。

    固定帧头和帧尾帮助MSPM0从连续字节流中重新找到帧边界。所有字段发送
    原始数值而非ASCII。例如TASK2稳定识别到3号房在左边，发送AA 03 01 01 02 55。
    尚无稳定结果时发送AA 00 00 00 TASK 55，MSPM0可据此区分“无结果”和旧结果。
    """
    frame = bytes((RESULT_FRAME_HEADER, room, side, found, task, RESULT_FRAME_TAIL))
    sent = 0

    # CanMV UART.write()返回实际写入的字节数，不能假定一次调用一定成功。
    # 每次只发一个字节并留出1ms，使MSPM0即使依靠主循环轮询也不会溢出RX FIFO。
    for value in frame:
        start_ms = time.ticks_ms()
        one_byte = bytes((value,))
        while True:
            written = uart.write(one_byte)
            if written is not None and written == 1:
                sent += 1
                break

            if time.ticks_diff(time.ticks_ms(), start_ms) >= UART_WRITE_TIMEOUT_MS:
                print("UART TX timeout: %d/%d bytes" % (sent, len(frame)))
                return False
            time.sleep_ms(1)

        time.sleep_ms(UART_INTER_BYTE_MS)

    # print只发送到CanMV IDE调试终端，不会混入UART2数据。
    print(
        "TX -> MSPM0: task %d, found %d, room %d, side %s, bytes %d/%d"
        % (task, found, room, SIDE_NAMES[side], sent, len(frame))
    )
    return True


def main():
    """初始化外设和模型，持续完成取图、识别、显示、消抖及串口发送。"""
    # 先设为None，确保初始化中途失败时finally只释放已经创建成功的资源。
    pipeline = None
    detector = None
    uart = None

    # 在初始化摄像头和KPU之前检查模型，便于快速发现SD卡路径或复制错误。
    if not file_exists(KMODEL_PATH):
        print("ERROR: kmodel not found: %s" % KMODEL_PATH)
        return

    try:
        # 先初始化通信，即使后面模型加载失败，也可以从日志确认UART配置已执行。
        uart = init_uart()

        # PipeLine负责创建摄像头AI图像通道、LCD显示层和OSD叠加图层。
        pipeline = PipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_mode=DISPLAY_MODE,
            display_size=DISPLAY_SIZE,
        )
        # 真正申请摄像头、显示和媒体缓冲区资源。
        pipeline.create()
        # 获取LCD实际宽高；后续YOLO框坐标和左右中线判断都使用这个坐标系。
        display_size = pipeline.get_display_size()
        print("LCD size: %dx%d" % (display_size[0], display_size[1]))

        try:
            # 创建实时目标检测器。这里不会重新训练模型，只是加载best.kmodel进行推理。
            detector = DigitYOLOv8(
                # detect表示目标检测，会同时输出类别、位置框和置信度。
                task_type="detect",
                # video表示连续摄像头视频模式，而不是单张图片模式。
                mode="video",
                kmodel_path=KMODEL_PATH,
                labels=LABELS,
                rgb888p_size=RGB888P_SIZE,
                model_input_size=MODEL_INPUT_SIZE,
                display_size=display_size,
                conf_thresh=CONFIDENCE_THRESHOLD,
                nms_thresh=NMS_THRESHOLD,
                max_boxes_num=MAX_BOXES_NUM,
                # 设为1或更高可打印更多耗时信息，但会影响调试输出和少量性能。
                debug_mode=0,
            )
        except Exception as error:
            print("ERROR: kmodel load failed")
            print("Check that CanMV firmware and kmodel nncase versions match.")
            raise error

        # 配置并构建前面自定义的AI2D letterbox预处理管线，只需执行一次。
        detector.config_preprocess()

        # 双向串口命令解析器：主循环每帧轮询一次，不会阻塞摄像头推理。
        command_parser = CommandParser()
        # 上电默认执行TASK1；MSPM0随后可以随时通过命令帧切换任务。
        current_task = DEFAULT_TASK
        target_room = DEFAULT_TARGET_ROOM
        # 单独记录串口实际收到的命令，确保首个TASK1即使与默认状态相同也会打印一次。
        last_received_command = None

        # candidate_result：当前正在观察、但尚未达到稳定帧数的(room, side)。
        candidate_result = None
        # candidate_frames：候选结果连续出现的帧数。
        candidate_frames = 0
        # missing_frames：连续没有有效1~8检测结果的帧数。
        missing_frames = 0
        # confirmed_result：通过连续帧确认、允许通过UART发送的(room, side)。
        confirmed_result = None
        # last_sent_result：上一次已发送的(room, side, found, task)，用于检测状态变化。
        last_sent_result = None
        # last_send_ms：上次发送时刻，用于实现200ms心跳重发。
        last_send_ms = time.ticks_ms()

        # 以下三个变量用于计算并显示每秒处理帧数FPS，不参与识别判断。
        frame_count = 0
        fps = 0
        fps_start_ms = time.ticks_ms()

        while True:
            # 允许CanMV IDE通过停止按钮安全打断无限循环。
            os.exitpoint()

            # ---------------- 接收MSPM0视觉任务命令 ----------------
            # poll()为非阻塞函数，没有新命令时立即返回None，不影响当前视觉任务继续运行。
            command = command_parser.poll(uart)
            if command is not None:
                new_task, new_target_room = command

                # 首次收到以及命令内容变化时打印；周期性重发的相同命令不重复刷屏。
                if command != last_received_command:
                    print(
                        "RX <- MSPM0: task %d (%s), target %d"
                        % (new_task, TASK_NAMES[new_task], new_target_room)
                    )
                    last_received_command = command

                # 只有任务或目标真正变化时才重置识别状态；重复命令不会反复清空结果。
                if new_task != current_task or new_target_room != target_room:
                    current_task = new_task
                    target_room = new_target_room

                    # 新任务必须重新累计稳定帧，禁止把上一任务结果带入新任务。
                    candidate_result = None
                    candidate_frames = 0
                    missing_frames = 0
                    confirmed_result = None
                    last_sent_result = None

            # 从摄像头AI通道取得一帧RGB888P图像。
            frame = pipeline.get_frame()
            # 执行AI2D预处理、KPU推理和YOLO NMS后处理，得到检测框/类别/置信度。
            result = detector.run(frame)
            # 按TASK1/TASK2的规则筛选检测框；TASK2只会选择target_room对应的数字。
            room, side, score = select_vision_result(
                result,
                current_task,
                target_room,
                display_size[0],
            )

            # 使用ticks_diff而不是直接相减，可正确处理MicroPython毫秒计数器回绕。
            frame_count += 1
            now = time.ticks_ms()
            elapsed = time.ticks_diff(now, fps_start_ms)
            if elapsed >= 1000:
                fps = frame_count * 1000 // elapsed
                frame_count = 0
                fps_start_ms = now

            # 先让YOLO类把全部检测框绘制到透明OSD图层。
            detector.draw_result(result, pipeline.osd_img)
            # 再在左上角叠加任务、目标、本程序最终选择的房号/方向以及FPS。
            if room is None:
                if current_task == TASK_INITIAL_ROOM:
                    status = "T1 INITIAL Room:-- FPS:%d" % fps
                else:
                    status = "T2 FIND:%d Room:-- FPS:%d" % (target_room, fps)
                status_color = (255, 255, 255)
            else:
                status = "T%d Find:%d Room:%d %s %.2f FPS:%d" % (
                    current_task,
                    target_room,
                    room,
                    SIDE_NAMES[side],
                    score,
                    fps,
                )
                status_color = (0, 255, 0)
            pipeline.osd_img.draw_string_advanced(
                8, 8, 28, status, color=status_color
            )
            # 将摄像头画面与OSD叠加层送到LCD显示。
            pipeline.show_image()

            # ---------------- 连续帧消抖/稳定确认 ----------------
            if room is None:
                # 本帧没有有效1~8。候选连续性被打断，所以立即清空候选计数。
                missing_frames += 1
                candidate_result = None
                candidate_frames = 0
                if missing_frames == UART_CLEAR_FRAMES:
                    # 连续丢失达到阈值后，旧确认结果失效。
                    confirmed_result = None
            else:
                # 有效检测会打断“连续丢失”计数。
                missing_frames = 0
                # 房号和方向作为一个整体消抖。房号相同但LEFT变RIGHT，也重新计数。
                current_result = (room, side)
                if current_result == candidate_result:
                    candidate_frames += 1
                else:
                    # 结果变化，开始统计这个新候选连续出现的帧数。
                    candidate_result = current_result
                    candidate_frames = 1

                if candidate_frames >= UART_STABLE_FRAMES:
                    # 连续达到5帧后才允许发送，过滤偶发单帧误识别和方向跳变。
                    confirmed_result = current_result

            # ---------------- UART结果发送/心跳重发 ----------------
            # 无稳定结果也会发送FOUND=0，使MSPM0能明确知道当前任务仍未识别成功。
            if confirmed_result is None:
                send_payload = (0, SIDE_CENTER, 0, current_task)
            else:
                send_payload = (
                    confirmed_result[0],
                    confirmed_result[1],
                    1,
                    current_task,
                )

            now = time.ticks_ms()
            if (
                # 房号、方向、成功标志或任务变化时立即发送。
                send_payload != last_sent_result
                # 状态不变时每200ms重发，使MSPM0漏掉单帧后仍有机会收到。
                or time.ticks_diff(now, last_send_ms) >= UART_HEARTBEAT_MS
            ):
                send_ok = send_result(
                    uart,
                    send_payload[0],
                    send_payload[1],
                    send_payload[2],
                    send_payload[3],
                )
                if send_ok:
                    last_sent_result = send_payload
                    last_send_ms = now

            # 主动回收本帧产生的临时Python对象，减少长时间视频推理的内存碎片。
            gc.collect()

    except KeyboardInterrupt:
        # 用户在CanMV IDE点击停止时通常进入这里，这是正常退出而不是程序故障。
        print("Stopped by user")
    except Exception as error:
        # 打印完整MicroPython异常堆栈，方便定位模型、内存或媒体初始化问题。
        sys.print_exception(error)
    finally:
        # 无论正常停止还是发生异常，都按“模型 -> 媒体管线 -> UART”顺序释放资源。
        # 这对CanMV IDE中反复运行脚本尤其重要，否则下一次可能因资源未释放而失败。
        if detector is not None:
            detector.deinit()
        if pipeline is not None:
            pipeline.destroy()
        if uart is not None:
            uart.deinit()
        gc.collect()


if __name__ == "__main__":
    # 只有直接运行本文件时才进入main；被其他脚本import时不会自动启动摄像头。
    main()
