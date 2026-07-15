#!/usr/bin/env python3

import cv2

from yolo_detector import YoloDetector
from pointer_reader import PointerDialReader
from pointer_value_convertor import PointerValueConvertor
from tts_player import TTS

if __name__ == "__main__":
    # 仪表盘参数
    dashboard_params = {
        'min_angle': 225,  # 仪表最小量程时，指针角度（指针竖直向上是为0，顺时针旋转）
        'max_angle': 135,  # 仪表最大量程时，指针角度（指针竖直向上是为0，顺时针旋转）
        'min_value': 0,  # 仪表最小量程
        'max_value': 2.5,  # 仪表最大量程
    }

    # 初始化 YOLO 检测器
    yolo_detector = YoloDetector()

    # 初始化表盘读取器
    dial_reader = PointerDialReader()

    # 初始化指针数据转换
    convertor = PointerValueConvertor(
        dashboard_params['min_angle'],
        dashboard_params['max_angle'],
        dashboard_params['min_value'],
        dashboard_params['max_value']
    )

    # 初始化相机
    cap = cv2.VideoCapture('http://0.0.0.0:8080/stream?topic=/camera/color/image_raw')

    # YOLO 预读取帧数设置
    loaded_time = 0
    pre_load_limit = yolo_detector.get_worker_number() + 1

    model_path = "/home/bcsh/Documents/Models/tts/vits-zh-hf-fanchen-C"
    tts_player = TTS(model_dir=model_path, debug=False)

    while True:
        # 读取相机数据
        ret, frame = cap.read()
        if not ret:
            raise RuntimeError("无法打开摄像头，请检查线路连接")

        # 初始化异步所需要的帧
        if loaded_time < pre_load_limit:
            yolo_detector.fill_pool(frame)
            loaded_time += 1
            continue

        # 检测表盘
        detections = yolo_detector.process_frame(frame)

        # 跳过没有检测到表盘的数据
        if len(detections) < 1:
            continue

        # 解包检测后的数据
        name, score, watcher_center, boundary = detections[0]
        x1, y1, x2, y2 = boundary

        # 跳过识别有问题的帧
        if (x2 - x1) < 0 or (y2 - y1) < 0:
            continue

        # 截取表盘区域
        watcher_roi = frame[y1:y2, x1:x2]
        
        if watcher_roi is not None:
            # 检测表盘中心和半径
            center, radius = dial_reader.find_dial_center(watcher_roi)
            # print(f"center: {center}, radius: {radius}")

        if center is not None:
            cv2.circle(watcher_roi, center, 4, (0, 0, 255), 2)
            cv2.circle(watcher_roi, center, radius, (0, 0, 255), 2)

        # 检测指针
        pointer = dial_reader.detect_pointer(watcher_roi, center, radius)
        # print(f"pointer: {pointer}")

        if pointer is not None:
            cv2.line(watcher_roi, (pointer[0], pointer[1]), (pointer[2], pointer[3]), (255, 0, 0), 2)

        # 跳过未检测到表盘中心和指针的帧
        if center is None or pointer is None:
            continue

        # 计算指针角度
        angle = dial_reader.calculate_angle(pointer, center)
        print(f"angle: {angle}")

        # 将指针角度转换为表盘上的读数
        value = convertor.angle_to_value(angle)
        print(f"value: {value}")

        # 显示读数,小于1.65绿色显示，代表正常
        if 0 < value < 1.65:
            cv2.putText(watcher_roi,text=f"{value}:Normal",org=(10, 30),fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=1.0,color=(0, 255, 0),thickness=2)
            cv2.imwrite("/home/bcsh/panel_image.jpg", watcher_roi)
        # 显示读数，大于1.65红色显示，代表不正常
        elif 1.65 <= value <= 2.5:
            cv2.putText(watcher_roi,text=f"{value}:Warning",org=(10, 30),fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=1.0,color=(0, 0, 255),thickness=2)
            cv2.imwrite("/home/bcsh/panel_image.jpg", watcher_roi)
            tts_player.generate_and_play('仪表盘读数过高，请注意危险')         
        else:
            cv2.putText(watcher_roi,text="unknown",org=(10, 30),fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=1.0,color=(255, 0, 0),thickness=2)
       
        cv2.namedWindow("watcher_roi", cv2.WINDOW_NORMAL)
        cv2.imshow("watcher_roi", watcher_roi)
