"""Backup: K230 steel-ball detection + UART control + RTSP + annotated LCD preview.

Design priority:
    1. AI channel (camera channel 0) runs every available camera frame.
    2. UART sends one position packet after every inference.
    3. Camera channel 1 is shared by hardware H.264 RTSP and the onboard LCD.
    4. Detection boxes/status are drawn only on the LCD OSD layer.

CanMV v1.x channel rule:
    A channel bound to VENC must have a higher channel number than a channel
    used by snapshot().  Binding channel 0 and snapshotting channel 2 causes
    RuntimeError: sensor(0) snapshot chn(2) failed(3).  Therefore this program
    deliberately uses snapshot channel 0 and bound VENC channel 1.

The RTSP image is the raw camera image and does not contain detection boxes.
This avoids drawing/copying/encoding in the AI loop and protects control FPS.

UART packet (8 bytes):
    AA SEQ VALID POS_H POS_L CONF CHECKSUM 55

POS is the ball position along the tube, normalized to 0..1000.  When VALID
is zero, POS is 0xFFFF.  CHECKSUM is the low byte of bytes 1 through 5.
"""

from libs.YOLO import YOLOv8
from libs.Utils import ScopedTiming, letterbox_pad_param
from machine import FPIOA, UART
from media.sensor import (
    Sensor,
    CAM_CHN_ID_0,
    CAM_CHN_ID_1,
    PIXEL_FORMAT_YUV_SEMIPLANAR_420,
    PIXEL_FORMAT_RGB_888_PLANAR,
)
from media.display import Display
from media.vencoder import Encoder, ChnAttrStr, StreamData, VENC_CHN_ID_0
from media.media import (
    ALIGN_UP,
    MediaManager,
    VIDEO_ENCODE_MOD_ID,
    VENC_DEV_ID,
)

import nncase_runtime as nn
import multimedia as mm
import network
import image
import _thread
import uctypes
import gc
import os
import sys
import time


# ---------------------------------------------------------------------------
# User configuration
# ---------------------------------------------------------------------------

KMODEL_PATH = "/sdcard/apps/ball_detect/ball.kmodel"
LABELS = ["ball"]
MODEL_INPUT_SIZE = [320, 320]

# AI acquisition size.  Keep 640x360 first; reducing this can increase FPS but
# may reduce the precision of the ball center coordinate.
AI_IMAGE_SIZE = [640, 360]
CAMERA_FPS = 30

CONFIDENCE_THRESHOLD = 0.25
NMS_THRESHOLD = 0.45
MAX_BOXES_NUM = 5

# 0: the tube is horizontal in the image; 1: the tube is vertical.
TRACK_AXIS = 0
# Change to True when the reported 0..1000 direction is reversed.
REVERSE_POSITION = False

# Lushan Pi UART2: K230 GPIO11 TX -> MSPM0 RX; both boards must share GND.
UART_TX_PIN = 11
UART_RX_PIN = 12
UART_BAUDRATE = 115200

# Wi-Fi must be 2.4 GHz.  Change these two strings before running.
WIFI_SSID = "zhao"
WIFI_PASSWORD = "12345679"
WIFI_TIMEOUT_S = 15

# If Wi-Fi is unavailable, the program continues detecting and sending UART.
ENABLE_RTSP = True
RTSP_SESSION = "ball"
RTSP_PORT = 8554

# Restore the directly-bound stream profile that was previously verified on
# this board.  Channel 1 is connected straight to the hardware H.264 encoder;
# Python does not snapshot, resize or software-encode the stream image.
STREAM_WIDTH = 640
STREAM_HEIGHT = 480
STREAM_FPS = 30
STREAM_BUFFER_COUNT = 15

# Lushan Pi onboard ST7701 LCD.  The raw image shares camera channel 1 with
# RTSP; only the transparent detection OSD is refreshed from Python.
# Keep to_ide=False because IDE JPEG preview costs extra CPU and memory.
ENABLE_LCD = True
LCD_WIDTH = 800
LCD_HEIGHT = 480

FRAME_HEADER = 0xAA
FRAME_TAIL = 0x55


class BallYOLOv8(YOLOv8):
    """YOLOv8 with letterbox preprocessing matching the exported kmodel."""

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = (
                input_image_size if input_image_size else self.rgb888p_size
            )
            top, bottom, left, right, self.scale = letterbox_pad_param(
                self.rgb888p_size, self.model_input_size
            )
            self.ai2d.pad(
                [0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114]
            )
            self.ai2d.resize(
                nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel
            )
            self.ai2d.build(
                [1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                [1, 3, self.model_input_size[1], self.model_input_size[0]],
            )


class SharedCameraRtsp:
    """RTSP sender sharing channel 1 of the AI camera.

    Unlike media.rtspserver.RtspServer, this class does not create/reset a
    second Sensor and does not call MediaManager.init/deinit.  This is crucial:
    the AI and stream must share one VICAP/ISP pipeline.
    """

    def __init__(self, sensor):
        self.sensor = sensor
        self.venc_chn = VENC_CHN_ID_0
        self.encoder = Encoder()
        self.link = None
        self.server = mm.rtsp_server()
        self.running = False
        self.thread_over = True
        self.encoder_created = False
        self.server_started = False

    def reserve_buffers_and_link(self):
        """Call before MediaManager.init()."""
        width = ALIGN_UP(STREAM_WIDTH, 16)
        self.encoder.SetOutBufs(
            self.venc_chn,
            STREAM_BUFFER_COUNT,
            width,
            STREAM_HEIGHT,
        )
        self.link = MediaManager.link(
            self.sensor.bind_info(chn=CAM_CHN_ID_1)["src"],
            (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, self.venc_chn),
        )

    def start_after_media_init(self):
        """Create VENC/RTSP after MediaManager.init(), before sensor.run()."""
        width = ALIGN_UP(STREAM_WIDTH, 16)
        attr = ChnAttrStr(
            self.encoder.PAYLOAD_TYPE_H264,
            self.encoder.H264_PROFILE_MAIN,
            width,
            STREAM_HEIGHT,
            STREAM_FPS,  # GOP length; one I-frame per second at 30 FPS.
        )
        self.encoder.Create(self.venc_chn, attr)
        self.encoder_created = True

        self.server.rtspserver_init(RTSP_PORT)
        self.server.rtspserver_createsession(
            RTSP_SESSION,
            mm.multi_media_type.media_h264,
            False,
        )
        self.server.rtspserver_start()
        self.server_started = True
        self.encoder.Start(self.venc_chn)

    def start_sender_thread(self):
        self.thread_over = False
        self.running = True
        _thread.start_new_thread(self._send_loop, ())

    def get_url(self):
        return self.server.rtspserver_getrtspurl(RTSP_SESSION)

    def _send_loop(self):
        """Only this thread touches the encoded stream/network sender."""
        stream_data = StreamData()
        try:
            while self.running:
                # GetStream blocks until VENC has an encoded frame.  VENC runs
                # in hardware and is fed directly by camera channel 1.
                self.encoder.GetStream(self.venc_chn, stream_data)
                try:
                    for pack_index in range(stream_data.pack_cnt):
                        packet = bytes(
                            uctypes.bytearray_at(
                                stream_data.data[pack_index],
                                stream_data.data_size[pack_index],
                            )
                        )
                        self.server.rtspserver_sendvideodata(
                            RTSP_SESSION,
                            packet,
                            stream_data.data_size[pack_index],
                            1000,
                        )
                finally:
                    self.encoder.ReleaseStream(self.venc_chn, stream_data)
        except BaseException as error:
            # A broken Wi-Fi link must not stop ball detection/control.
            print("RTSP sender stopped; AI/UART continue:")
            sys.print_exception(error)
        finally:
            self.running = False
            self.thread_over = True

    def stop(self):
        """Stop the sender/server/encoder.  Sensor and MediaManager are external."""
        self.running = False

        # GetStream should return on the next camera frame.  Do not wait forever
        # during a Ctrl+C cleanup if the Wi-Fi or encoder is already unhealthy.
        deadline = time.ticks_add(time.ticks_ms(), 1500)
        while not self.thread_over and time.ticks_diff(deadline, time.ticks_ms()) > 0:
            time.sleep_ms(20)

        if self.server_started:
            try:
                self.server.rtspserver_stop()
                self.server.rtspserver_deinit()
            except BaseException as error:
                print("RTSP server cleanup warning:")
                sys.print_exception(error)
            self.server_started = False

        if self.encoder_created:
            try:
                self.encoder.Stop(self.venc_chn)
                self.encoder.Destroy(self.venc_chn)
            except BaseException as error:
                print("VENC cleanup warning:")
                sys.print_exception(error)
            self.encoder_created = False

        if self.link is not None:
            del self.link
            self.link = None


def file_exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def connect_wifi():
    """Connect to the 2.4 GHz AP and wait for a real DHCP address."""
    station = network.WLAN(network.STA_IF)
    config = station.ifconfig()
    if station.isconnected() and config[0] != "0.0.0.0":
        return station

    print("Connecting to 2.4 GHz Wi-Fi:", WIFI_SSID)
    # Do not call active(True): Lushan Pi CanMV v1.2.2 reports that changing
    # the RT-Smart network active state is unsupported.
    station.connect(WIFI_SSID, WIFI_PASSWORD)
    deadline = time.ticks_add(time.ticks_ms(), WIFI_TIMEOUT_S * 1000)
    retry_at = time.ticks_add(time.ticks_ms(), 5000)

    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        config = station.ifconfig()
        if station.isconnected() and config[0] != "0.0.0.0":
            print("Wi-Fi ready:", config)
            return station

        if time.ticks_diff(time.ticks_ms(), retry_at) >= 0:
            station.connect(WIFI_SSID, WIFI_PASSWORD)
            retry_at = time.ticks_add(time.ticks_ms(), 5000)
        time.sleep_ms(200)
        os.exitpoint()

    raise RuntimeError("Wi-Fi/DHCP timeout: " + str(station.ifconfig()))


def init_uart():
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)
    return UART(
        UART.UART2,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )


def best_detection(result):
    """Return the highest-confidence box, not just the first YOLO box."""
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


def send_position(uart, sequence, valid, position, confidence):
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
            [
                FRAME_HEADER,
                sequence,
                int(valid),
                position_high,
                position_low,
                confidence_byte,
                checksum,
                FRAME_TAIL,
            ]
        )
    )


def configure_sensor():
    """Configure one camera with only two outputs: AI and shared video."""
    sensor = Sensor(fps=CAMERA_FPS)
    sensor.reset()

    # Channel 0: planar RGB -> ai2d/KPU.  Keeping snapshot on the lowest
    # channel is required by the CanMV v1.x multi-channel VICAP pipeline.
    sensor.set_framesize(
        w=ALIGN_UP(AI_IMAGE_SIZE[0], 16),
        h=AI_IMAGE_SIZE[1],
        chn=CAM_CHN_ID_0,
    )
    sensor.set_pixformat(
        PIXEL_FORMAT_RGB_888_PLANAR,
        chn=CAM_CHN_ID_0,
    )

    # Channel 1: YUV -> VENC and onboard LCD hardware.  Python never snapshots
    # this channel, so displaying/streaming the raw image does not copy frames.
    sensor.set_framesize(
        w=ALIGN_UP(STREAM_WIDTH, 16),
        h=STREAM_HEIGHT,
        chn=CAM_CHN_ID_1,
        alignment=12,
    )
    sensor.set_pixformat(
        PIXEL_FORMAT_YUV_SEMIPLANAR_420,
        chn=CAM_CHN_ID_1,
    )

    # Do not call the private _set_chn_fps() API here.  On CanMV v1.2.2 it can
    # make the camera channels run with unstable timing.  Both channels use the
    # sensor's native 30 FPS; VENC and LCD remain direct hardware paths.
    return sensor


def main():
    sensor = None
    detector = None
    uart = None
    rtsp = None
    media_initialized = False
    display_initialized = False
    osd_img = None
    station = None

    try:
        if not file_exists(KMODEL_PATH):
            raise RuntimeError("Model not found: " + KMODEL_PATH)

        os.exitpoint(os.EXITPOINT_ENABLE)
        nn.shrink_memory_pool()
        uart = init_uart()

        # Wi-Fi failure is nonfatal because accurate control has priority.
        rtsp_enabled = ENABLE_RTSP
        if rtsp_enabled:
            try:
                station = connect_wifi()
            except BaseException as error:
                rtsp_enabled = False
                print("Wi-Fi unavailable; starting AI/UART without RTSP:")
                sys.print_exception(error)

        sensor = configure_sensor()

        # The LCD and VENC both consume channel 1.  The 640-pixel video is
        # centered on the 800-pixel ST7701; OSD remains full-screen.
        if ENABLE_LCD:
            lcd_x = (LCD_WIDTH - STREAM_WIDTH) // 2
            lcd_y = (LCD_HEIGHT - STREAM_HEIGHT) // 2
            lcd_bind_info = sensor.bind_info(
                x=lcd_x,
                y=lcd_y,
                chn=CAM_CHN_ID_1,
            )
            Display.bind_layer(
                **lcd_bind_info,
                layer=Display.LAYER_VIDEO1
            )
            Display.init(
                Display.ST7701,
                width=LCD_WIDTH,
                height=LCD_HEIGHT,
                to_ide=False,
            )
            display_initialized = True
            osd_img = image.Image(LCD_WIDTH, LCD_HEIGHT, image.ARGB8888)

        # VENC output buffers and the camera->VENC link must be registered
        # before the single MediaManager.init() call.
        if rtsp_enabled:
            rtsp = SharedCameraRtsp(sensor)
            rtsp.reserve_buffers_and_link()

        MediaManager.init()
        media_initialized = True

        if rtsp is not None:
            rtsp.start_after_media_init()

        detector = BallYOLOv8(
            task_type="detect",
            mode="video",
            kmodel_path=KMODEL_PATH,
            labels=LABELS,
            rgb888p_size=AI_IMAGE_SIZE,
            model_input_size=MODEL_INPUT_SIZE,
            # draw_result() maps AI coordinates to this LCD/OSD resolution.
            display_size=[LCD_WIDTH, LCD_HEIGHT],
            conf_thresh=CONFIDENCE_THRESHOLD,
            nms_thresh=NMS_THRESHOLD,
            max_boxes_num=MAX_BOXES_NUM,
            debug_mode=0,
        )
        detector.config_preprocess()

        sensor.run()
        # Let the ISP/VICAP pipeline produce stable frames before the sender
        # thread starts consuming encoded channel-1 frames.
        time.sleep_ms(100)
        warmup_ok = False
        for warmup_attempt in range(10):
            try:
                sensor.snapshot(chn=CAM_CHN_ID_0)
                warmup_ok = True
                break
            except RuntimeError:
                time.sleep_ms(20)
        if not warmup_ok:
            raise RuntimeError("AI camera channel 0 warmup failed")
        if rtsp is not None:
            rtsp.start_sender_thread()
            board_ip = station.ifconfig()[0]
            print("RTSP URL: rtsp://{}:{}/{}".format(
                board_ip, RTSP_PORT, RTSP_SESSION
            ))
            print("RTSP raw H.264: {}x{}, target {} FPS".format(
                STREAM_WIDTH, STREAM_HEIGHT, STREAM_FPS
            ))

        print("Ball AI/UART started: UART2 115200 8N1")
        if display_initialized:
            print("Onboard LCD preview with AI boxes: {}x{}".format(
                LCD_WIDTH, LCD_HEIGHT
            ))

        sequence = 0
        frame_counter = 0
        snapshot_failures = 0
        fps_counter = 0
        fps_start = time.ticks_ms()

        while True:
            os.exitpoint()

            # Only AI channel 0 enters Python.  RTSP channel 1 is bound to VENC.
            # A transient snapshot timeout must not terminate UART control.
            try:
                frame = sensor.snapshot(chn=CAM_CHN_ID_0)
                snapshot_failures = 0
            except RuntimeError as error:
                snapshot_failures += 1
                if snapshot_failures == 1 or (snapshot_failures % 30) == 0:
                    print("AI snapshot retry", snapshot_failures, error)
                time.sleep_ms(5)
                continue
            image_array = frame.to_numpy_ref()
            result = detector.run(image_array)

            box, score = best_detection(result)
            if box is None:
                send_position(uart, sequence, False, 0, 0.0)
            else:
                center_x = float(box[0]) + float(box[2]) * 0.5
                center_y = float(box[1]) + float(box[3]) * 0.5
                if TRACK_AXIS == 0:
                    axis_pixel = center_x
                    axis_length = AI_IMAGE_SIZE[0]
                else:
                    axis_pixel = center_y
                    axis_length = AI_IMAGE_SIZE[1]

                position = int(axis_pixel * 1000.0 / axis_length + 0.5)
                position = max(0, min(1000, position))
                if REVERSE_POSITION:
                    position = 1000 - position
                send_position(uart, sequence, True, position, score)

            # Match ball_detect_test.py: draw boxes and status on the LCD OSD.
            # The RTSP channel remains raw and does not pay this drawing cost.
            if osd_img is not None:
                osd_img.clear()
                detector.draw_result(result, osd_img)
                if box is None:
                    status = "NO BALL"
                    status_color = (255, 80, 80)
                else:
                    status = "BALL %.2f  X:%d Y:%d" % (
                        score,
                        int(center_x),
                        int(center_y),
                    )
                    status_color = (0, 255, 0)
                osd_img.draw_string_advanced(
                    8, 8, 24, status, color=status_color
                )
                Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)

            sequence = (sequence + 1) & 0xFF
            fps_counter += 1
            frame_counter += 1

            now = time.ticks_ms()
            elapsed = time.ticks_diff(now, fps_start)
            if elapsed >= 1000:
                fps = fps_counter * 1000.0 / elapsed
                print("AI/control FPS: %.1f" % fps)
                fps_counter = 0
                fps_start = now

            # Less frequent GC than the display version to reduce control jitter.
            if (frame_counter & 63) == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("Stopped by user")
    except BaseException as error:
        sys.print_exception(error)
    finally:
        # Stop RTSP while the sensor can still release a pending GetStream.
        if rtsp is not None:
            rtsp.stop()
        if sensor is not None:
            try:
                sensor.stop()
            except BaseException:
                pass
        if detector is not None:
            detector.deinit()
        # CanMV requires Display.deinit() after sensor.stop() and before
        # MediaManager.deinit().
        if display_initialized:
            try:
                Display.deinit()
            except BaseException:
                pass
        osd_img = None
        if media_initialized:
            MediaManager.deinit()
        if uart is not None:
            uart.deinit()
        gc.collect()
        print("Ball AI/RTSP stopped")


if __name__ == "__main__":
    main()
