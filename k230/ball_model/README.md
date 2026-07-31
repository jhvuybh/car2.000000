# 庐山派 K230 钢球识别测试

已生成的模型参数：

- 任务：YOLOv8 单类别目标检测
- 类别：`ball`
- 模型输入：`1 × 3 × 320 × 320`
- 模型输出：`1 × 5 × 2100`
- K230 模型：INT8 量化，校准集使用训练数据中的 100 张图像
- 当前权重来源：`runs/detect/ball1.0/weights/best.pt`（2026-07-30）

## 部署

在 K230 的 SD 卡中创建目录：

```text
/sdcard/apps/ball_detect/
```

将下面两个文件复制到该目录：

```text
ball.kmodel
ball_detect_test.py
```

最终路径必须是：

```text
/sdcard/apps/ball_detect/ball.kmodel
/sdcard/apps/ball_detect/ball_detect_test.py
```

在 CanMV IDE 中打开并运行 `ball_detect_test.py`。默认输出到庐山派板载
ST7701 LCD；如果使用 HDMI，把代码中的 `DISPLAY_MODE = "lcd"` 改为
`DISPLAY_MODE = "hdmi"`。

屏幕左上角显示：置信度、钢球中心坐标和 FPS；串口终端每 500 ms 打印
一次最佳识别结果。坐标原点在画面左上角，X 向右增加，Y 向下增加。

## 调参

若漏检较多，把 `CONFIDENCE_THRESHOLD` 从 `0.25` 降到 `0.15`；若误检
较多，将它提高到 `0.40` 或 `0.50`。测试时应尽量覆盖实际比赛中的水管、
灯光、摄像头高度和钢球尺寸。
