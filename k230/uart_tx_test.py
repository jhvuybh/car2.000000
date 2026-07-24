"""K230 -> MSPM0 单向UART发送测试，不加载摄像头、模型，也不等待MSPM0命令。"""

from machine import FPIOA, UART
import time


# False：UART2专用座T（逻辑GPIO11）。
# True：40Pin排针物理Pin11（逻辑GPIO5）。
UART_USE_40PIN_HEADER = False
UART_TX_PIN = 5 if UART_USE_40PIN_HEADER else 11
UART_BAUDRATE = 115200

# 模拟TASK1稳定识别到1号房：AA ROOM SIDE FOUND TASK 55
TEST_FRAME = bytes((0xAA, 0x01, 0x00, 0x01, 0x01, 0x55))

fpioa = FPIOA()
fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)

uart = UART(
    UART.UART2,
    baudrate=UART_BAUDRATE,
    bits=UART.EIGHTBITS,
    parity=UART.PARITY_NONE,
    stop=UART.STOPBITS_ONE,
)

try:
    print("UART2 TX test: GPIO%d -> MSPM0 PB16, 115200 8N1" % UART_TX_PIN)
    print("Sending: AA 01 00 01 01 55")
    while True:
        uart.write(TEST_FRAME)
        time.sleep_ms(200)
except KeyboardInterrupt:
    pass
finally:
    uart.deinit()
