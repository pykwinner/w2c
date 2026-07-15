from queue import Queue
from rknnlite.api import RKNNLite
from concurrent.futures import ThreadPoolExecutor, as_completed


class RKNNPoolExecutor:
    def __init__(
        self,
        func,
        rknn_model="./rknn_model/yolov5s.rknn",
        worker_number=1
    ):
        self.worker_number = worker_number
        self.rknn_model = rknn_model
        self.queue = Queue()
        self.rknn_pool = self.__init_rknn_pool()
        self.pool = ThreadPoolExecutor(max_workers=worker_number)
        self.func = func
        self.num = 0

    def __init_rknn_pool(self):
        rknn_pool = []
        for i in range(self.worker_number):
            rknn_lite = self.__init_rknn(i % 3)
            rknn_pool.append(rknn_lite)
        return rknn_pool

    def __init_rknn(self, id=0):
        rknn_lite = RKNNLite()
        ret = rknn_lite.load_rknn(self.rknn_model)
        if ret != 0:
            print("Load rknn_model failed")
            exit(ret)
        if id == 0:
            ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
        elif id == 1:
            ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_1)
        elif id == 2:
            ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_2)
        elif id == -1:
            ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
        else:
            ret = rknn_lite.init_runtime()
        if ret != 0:
            print("Init runtime environment failed")
            exit(ret)
        print(self.rknn_model, "\t\tdone")
        return rknn_lite

    def put(self, frame):
        self.queue.put(self.pool.submit(
            self.func, self.rknn_pool[self.num % self.worker_number], frame))
        self.num += 1

    def get(self):
        if self.queue.empty():
            return None, False
        fut = self.queue.get()
        return fut.result(), True

    def release(self):
        self.pool.shutdown()
        for rknn_lite in self.rknn_pool:
            rknn_lite.release()
