import cv2
import sys

from rknn_func_watch.rknn_pool import RKNNPoolExecutor
from rknn_func_watch.yolo_processor import YoloProcessor


class YoloDetector:
    def __init__(self):
        _model_path = sys.path[0] + "/rknn_model/watch-quantized-mmse.rknn"
        _yolo_processor = YoloProcessor()
        # 线程数, 增大可提高帧率
        self._worker_number = 4
        # 初始化 RKNN 池
        self._pool = RKNNPoolExecutor(
            rknn_model=_model_path, worker_number=self._worker_number, func=_yolo_processor.process)

    def __del__(self):
        self.clean_up()

    def get_worker_number(self):
        return self._worker_number

    def fill_pool(self, frame):
        frame = cv2.resize(frame, (640, 480))
        self._pool.put(frame)

    def process_frame(self, frame):
        self.fill_pool(frame)
        (frame, detections), flag = self._pool.get()
        result = frame.copy()

        cv2.imshow("Yolo Detect Image", result)
        cv2.waitKey(1)

        return detections

    def clear_pool(self):
        frame_queue = self._pool.queue
        while not frame_queue.empty():
            frame_queue.get()

    def clean_up(self):
        # 关闭OpenCV窗口
        cv2.destroyAllWindows()
        self._pool.release()


if __name__ == "__main__":

    # 设置一个窗口来显示图像
    result_name = "Yolo Detect Image"
    cv2.namedWindow(result_name, cv2.WINDOW_NORMAL)

    # 初始化 YOLO
    yolo_detector = YoloDetector()

    # 打开摄像头，使用默认摄像头（索引为 0）
    cap = cv2.VideoCapture(0)

    # 预加载次数
    loaded_time = 0
    pre_load_limit = yolo_detector.get_worker_number() + 1

    while True:

        ret, frame = cap.read()
        if not ret:
            raise RuntimeError("无法打开摄像头，请检查线路连接")

        if loaded_time >= pre_load_limit:

            # 处理数据
            detections = yolo_detector.process_frame(frame)

            print(f"detections: {detections}")

        else:

            # 初始化异步所需要的帧
            yolo_detector.process_frame(frame)
            loaded_time += 1
