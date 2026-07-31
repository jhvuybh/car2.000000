"""K230 / CanMV YOLOv8 steel-ball detection test.

Copy this file and ball.kmodel to:
    /sdcard/apps/ball_detect/

The program displays the camera image, detection boxes, ball center and FPS on
the Lushan Pi LCD.  Stop it with the Stop button in CanMV IDE.
"""

from libs.PipeLine import PipeLine
from libs.YOLO import YOLOv8
from libs.Utils import ScopedTiming, letterbox_pad_param
import nncase_runtime as nn
import gc
import os
import sys
import time


KMODEL_PATH = "/sdcard/apps/ball_detect/ball.kmodel"
LABELS = ["ball"]
MODEL_INPUT_SIZE = [320, 320]

# Lushan Pi onboard ST7701 LCD. Set this to "hdmi" when using HDMI output.
DISPLAY_MODE = "lcd"
DISPLAY_SIZE = None

# Camera image supplied to the AI pipeline. This size is fast enough for testing.
RGB888P_SIZE = [640, 360]

# Start with 0.25 while checking a new model. Raise it to 0.4~0.6 if false
# detections are frequent.
CONFIDENCE_THRESHOLD = 0.25
NMS_THRESHOLD = 0.45
MAX_BOXES_NUM = 10


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


def best_detection(result):
    """Return (box, score), choosing the highest-confidence ball."""
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

    try:
        if not file_exists(KMODEL_PATH):
            raise RuntimeError("Model not found: " + KMODEL_PATH)

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

        print("Ball detector started")
        print("Model:", KMODEL_PATH)
        print("Press Stop in CanMV IDE to quit")

        frame_count = 0
        fps = 0.0
        fps_start_ms = time.ticks_ms()
        last_print_ms = fps_start_ms

        while True:
            os.exitpoint()
            frame = pipeline.get_frame()
            result = detector.run(frame)

            # The CanMV YOLO helper draws every retained detection box.
            detector.draw_result(result, pipeline.osd_img)

            box, score = best_detection(result)
            frame_count += 1
            now_ms = time.ticks_ms()
            elapsed_ms = time.ticks_diff(now_ms, fps_start_ms)
            if elapsed_ms >= 1000:
                fps = frame_count * 1000.0 / elapsed_ms
                frame_count = 0
                fps_start_ms = now_ms

            if box is None:
                status = "NO BALL  FPS:%.1f" % fps
                color = (255, 80, 80)
            else:
                # CanMV YOLO detection boxes use [x, y, width, height].
                center_x = int(box[0] + box[2] * 0.5)
                center_y = int(box[1] + box[3] * 0.5)
                status = "BALL %.2f  X:%d Y:%d  FPS:%.1f" % (
                    score,
                    center_x,
                    center_y,
                    fps,
                )
                color = (0, 255, 0)

                # Print only twice per second so the serial terminal cannot
                # become the frame-rate bottleneck.
                if time.ticks_diff(now_ms, last_print_ms) >= 500:
                    print(
                        "ball score=%.3f center=(%d,%d) box=%s"
                        % (score, center_x, center_y, str(box))
                    )
                    last_print_ms = now_ms

            pipeline.osd_img.draw_string_advanced(
                8, 8, 24, status, color=color
            )
            pipeline.show_image()

            # Periodic collection is enough and costs less than collecting on
            # every frame.
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
        gc.collect()


if __name__ == "__main__":
    main()
