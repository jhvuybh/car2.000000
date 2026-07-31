"""K230 / CanMV YOLOv8 steel-ball detection with onboard LCD display.

Copy this file and the converted best.kmodel to:
    /sdcard/apps/ball_detect/

This version intentionally contains no Wi-Fi, RTSP or video encoder code.
"""

from libs.PipeLine import PipeLine
from libs.YOLO import YOLOv8
from libs.Utils import ScopedTiming, letterbox_pad_param
from machine import FPIOA, UART
import nncase_runtime as nn
import gc
import os
import sys
import time


KMODEL_PATH = "/sdcard/apps/ball_detect/best.kmodel"
LABELS = ["ball"]
MODEL_INPUT_SIZE = [320, 320]

# Lushan Pi onboard ST7701 LCD.
DISPLAY_MODE = "lcd"
DISPLAY_SIZE = None

# Keep the same camera/AI size as the version that was tested successfully.
RGB888P_SIZE = [640, 360]

CONFIDENCE_THRESHOLD = 0.25
NMS_THRESHOLD = 0.45
MAX_BOXES_NUM = 10

# LCD installation guide.  This line is always drawn at the exact center of
# the physical LCD and is independent of the adjustable balance-control zero.
SHOW_CENTER_GUIDE = True
CENTER_GUIDE_COLOR = (255, 255, 0)
CENTER_GUIDE_THICKNESS = 2

# ---------------------------------------------------------------------------
# Steel-ball coordinate UART output
# ---------------------------------------------------------------------------
# Wiring:
#   K230 GPIO11 (UART2_TX) -> MSPM0 PB16 (K230_INST RX)
#   K230 GND               -> MSPM0 GND
# No K230 RX wire is required because this program only transmits coordinates.
UART_TX_PIN = 11
UART_BAUDRATE = 115200

# Main center calibration value (AI image coordinate, range 0..639).
# When the ball is physically at point O, read X on the LCD and enter it here.
# Current real installation calibration: ball at physical center O -> X=394.
BALL_CENTER_PIXEL_X = 394.0

# MSPM0 protocol coordinate corresponding to physical center O.  Normally this
# stays at 500; only BALL_CENTER_PIXEL_X needs adjustment during installation.
BALL_CENTER_POSITION = 500.0

# Real three-point calibration (the physical positive direction is toward
# decreasing image X):
#   physical +5 cm -> X=234: (394-234)/5 = 32.0 pixels/cm
#   physical -5 cm -> X=542: (542-394)/5 = 29.6 pixels/cm
# After conversion X=234/394/542 maps exactly to P=700/500/300.
BALL_PIXELS_PER_CM_POSITIVE = 32.0
BALL_PIXELS_PER_CM_NEGATIVE = 29.6

FRAME_HEADER = 0xAA
FRAME_TAIL = 0x55


class BallYOLOv8(YOLOv8):
    """Use Ultralytics-compatible letterbox preprocessing."""

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = (
                input_image_size if input_image_size else self.rgb888p_size
            )
            top, bottom, left, right, self.scale = letterbox_pad_param(
                self.rgb888p_size, self.model_input_size
            )
            self.ai2d.pad(
                [0, 0, 0, 0, top, bottom, left, right],
                0,
                [114, 114, 114],
            )
            self.ai2d.resize(
                nn.interp_method.tf_bilinear,
                nn.interp_mode.half_pixel,
            )
            self.ai2d.build(
                [1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                [1, 3, self.model_input_size[1], self.model_input_size[0]],
            )


def file_exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def init_position_uart():
    """Configure UART2 TX on GPIO11 for the MSPM0 position receiver."""
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
    return UART(
        UART.UART2,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )


def pixel_x_to_position(center_x):
    """Convert X to P; physical + direction is toward decreasing image X."""
    center_x = float(center_x)
    if center_x < BALL_CENTER_PIXEL_X:
        position = BALL_CENTER_POSITION + (
            (BALL_CENTER_PIXEL_X - center_x) *
            40.0 / BALL_PIXELS_PER_CM_POSITIVE
        )
    else:
        position = BALL_CENTER_POSITION - (
            (center_x - BALL_CENTER_PIXEL_X) *
            40.0 / BALL_PIXELS_PER_CM_NEGATIVE
        )
    return max(0, min(1000, int(position + 0.5)))


def send_position(uart, sequence, valid, position=0, confidence=0.0):
    """Send: AA SEQ VALID POS_H POS_L CONF CHECKSUM 55."""
    if valid:
        position = max(0, min(1000, int(position)))
        confidence_byte = max(0, min(100, int(confidence * 100.0 + 0.5)))
    else:
        position = 0xFFFF
        confidence_byte = 0

    position_high = (position >> 8) & 0xFF
    position_low = position & 0xFF
    checksum = (
        sequence + int(valid) + position_high + position_low + confidence_byte
    ) & 0xFF
    uart.write(
        bytes(
            (
                FRAME_HEADER,
                sequence,
                int(valid),
                position_high,
                position_low,
                confidence_byte,
                checksum,
                FRAME_TAIL,
            )
        )
    )


def best_detection(result):
    """Return the highest-confidence detected steel ball."""
    if result is None or len(result) < 3 or len(result[0]) == 0:
        return None, 0.0

    best_index = 0
    best_score = float(result[2][0])
    for index in range(1, len(result[2])):
        score = float(result[2][index])
        if score > best_score:
            best_index = index
            best_score = score
    return result[0][best_index], best_score


def main():
    pipeline = None
    detector = None
    position_uart = None

    try:
        if not file_exists(KMODEL_PATH):
            raise RuntimeError("Model not found: " + KMODEL_PATH)

        position_uart = init_position_uart()

        # Use the firmware's original PipeLine implementation.  It manages the
        # camera, ST7701 video layer, AI frame channel and media cleanup.
        pipeline = PipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_mode=DISPLAY_MODE,
            display_size=DISPLAY_SIZE,
        )
        pipeline.create()

        detector = BallYOLOv8(
            task_type="detect",
            mode="video",
            kmodel_path=KMODEL_PATH,
            labels=LABELS,
            rgb888p_size=RGB888P_SIZE,
            model_input_size=MODEL_INPUT_SIZE,
            display_size=pipeline.get_display_size(),
            conf_thresh=CONFIDENCE_THRESHOLD,
            nms_thresh=NMS_THRESHOLD,
            max_boxes_num=MAX_BOXES_NUM,
            debug_mode=0,
        )
        detector.config_preprocess()

        display_size = pipeline.get_display_size()
        # Always place the installation guide at the exact LCD center.
        center_guide_x = display_size[0] // 2

        print("Ball detector started")
        print("Model:", KMODEL_PATH)
        print("Position UART: UART2 TX=GPIO11, 115200 8N1")
        print("Wireless video is handled by the external ESP32")
        print("Press Stop in CanMV IDE to quit")

        frame_count = 0
        fps = 0.0
        fps_start_ms = time.ticks_ms()
        last_print_ms = fps_start_ms
        sequence = 0

        while True:
            os.exitpoint()
            frame = pipeline.get_frame()
            result = detector.run(frame)

            # Draw every retained detection on the transparent LCD OSD layer.
            detector.draw_result(result, pipeline.osd_img)

            # Draw after draw_result(), because the detector clears/rebuilds
            # the OSD layer on every frame.  This vertical line marks the
            # optical center used when adjusting the camera installation.
            if SHOW_CENTER_GUIDE:
                pipeline.osd_img.draw_line(
                    center_guide_x,
                    0,
                    center_guide_x,
                    display_size[1] - 1,
                    color=CENTER_GUIDE_COLOR,
                    thickness=CENTER_GUIDE_THICKNESS,
                )

            box, score = best_detection(result)
            frame_count += 1
            now_ms = time.ticks_ms()
            elapsed_ms = time.ticks_diff(now_ms, fps_start_ms)
            if elapsed_ms >= 1000:
                fps = frame_count * 1000.0 / elapsed_ms
                frame_count = 0
                fps_start_ms = now_ms

            if box is None:
                send_position(position_uart, sequence, False)
                status = "NO BALL  FPS:%.1f" % fps
                color = (255, 80, 80)
            else:
                center_x = int(box[0] + box[2] * 0.5)
                center_y = int(box[1] + box[3] * 0.5)
                position = pixel_x_to_position(center_x)
                send_position(position_uart, sequence, True, position, score)
                status = "BALL %.2f X:%d P:%d FPS:%.1f" % (
                    score,
                    center_x,
                    position,
                    fps,
                )
                color = (0, 255, 0)

                # Limit terminal printing so it cannot become the FPS bottleneck.
                if time.ticks_diff(now_ms, last_print_ms) >= 500:
                    print(
                        "ball score=%.3f center=(%d,%d) box=%s"
                        % (score, center_x, center_y, str(box))
                    )
                    last_print_ms = now_ms

            sequence = (sequence + 1) & 0xFF

            pipeline.osd_img.draw_string_advanced(
                8, 8, 24, status, color=color
            )
            pipeline.show_image()

            if (frame_count & 15) == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("Stopped by user")
    except Exception as error:
        sys.print_exception(error)
    finally:
        if detector is not None:
            detector.deinit()
        if pipeline is not None:
            pipeline.destroy()
        if position_uart is not None:
            try:
                position_uart.deinit()
            except Exception:
                pass
        gc.collect()


if __name__ == "__main__":
    main()
