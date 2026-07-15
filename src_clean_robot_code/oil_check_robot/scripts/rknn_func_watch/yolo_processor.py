import cv2
import numpy as np


class YoloProcessor:
    def __init__(self):
        self.obj_threshold = 0.5
        self.nms_threshold = 0.45
        self.image_size = 640
        self.classes = ("watch",)

    def _roi_convert_to_diagonal(self, roi):
        """
        将 [x, y, w, h] 类型坐标转换为 [x1, y1, x2, y2] 类型坐标
        :param roi: [x, y, w, h] 类型坐标
        :return:
        """
        diagonal = np.copy(roi)
        diagonal[:, 0] = roi[:, 0] - roi[:, 2] / 2  # top left x
        diagonal[:, 1] = roi[:, 1] - roi[:, 3] / 2  # top left y
        diagonal[:, 2] = roi[:, 0] + roi[:, 2] / 2  # bottom right x
        diagonal[:, 3] = roi[:, 1] + roi[:, 3] / 2  # bottom right y
        return diagonal

    def _process(self, input, mask, anchors):
        anchors = [anchors[i] for i in mask]
        grid_h, grid_w = map(int, input.shape[0:2])

        box_confidence = input[..., 4]
        box_confidence = np.expand_dims(box_confidence, axis=-1)

        box_class_probs = input[..., 5:]

        box_xy = input[..., :2] * 2 - 0.5

        col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
        row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)
        col = col.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
        row = row.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
        grid = np.concatenate((col, row), axis=-1)
        box_xy += grid
        box_xy *= int(self.image_size / grid_h)

        box_wh = pow(input[..., 2:4] * 2, 2)
        box_wh = box_wh * anchors

        return np.concatenate((box_xy, box_wh), axis=-1), box_confidence, box_class_probs

    def _filter_boxes(self, boxes, box_confidences, box_class_probs):
        """
        Filter boxes with box threshold. It's a bit different with origin yolov5 post process!
        :param boxes:
        :param box_confidences:
        :param box_class_probs:
        :return:  boxes: ndarray, filtered boxes. classes: ndarray, classes for boxes. scores: ndarray, scores for boxes.
        """
        boxes = boxes.reshape(-1, 4)
        box_confidences = box_confidences.reshape(-1)
        box_class_probs = box_class_probs.reshape(-1, box_class_probs.shape[-1])

        _box_pos = np.where(box_confidences >= self.obj_threshold)
        boxes = boxes[_box_pos]
        box_confidences = box_confidences[_box_pos]
        box_class_probs = box_class_probs[_box_pos]

        class_max_score = np.max(box_class_probs, axis=-1)
        classes = np.argmax(box_class_probs, axis=-1)
        _class_pos = np.where(class_max_score >= self.obj_threshold)

        return boxes[_class_pos], classes[_class_pos], (class_max_score * box_confidences)[_class_pos]

    def _nms_boxes(self, boxes, scores):
        """
        Suppress non-maximal boxes.
        :param boxes: ndarray, boxes of objects.
        :param scores: ndarray, scores of objects.
        :return: keep: ndarray, index of effective boxes.
        """
        x = boxes[:, 0]
        y = boxes[:, 1]
        w = boxes[:, 2] - boxes[:, 0]
        h = boxes[:, 3] - boxes[:, 1]

        areas = w * h
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)

            xx1 = np.maximum(x[i], x[order[1:]])
            yy1 = np.maximum(y[i], y[order[1:]])
            xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
            yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

            w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
            h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
            inter = w1 * h1

            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            inds = np.where(ovr <= self.nms_threshold)[0]
            order = order[inds + 1]
        return np.array(keep)

    def _yolov5_post_process(self, input_data):
        masks = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
        anchors = [[10, 13], [16, 30], [33, 23], [30, 61], [62, 45],
                   [59, 119], [116, 90], [156, 198], [373, 326]]

        boxes, classes, scores = [], [], []
        for input, mask in zip(input_data, masks):
            b, c, s = self._process(input, mask, anchors)
            b, c, s = self._filter_boxes(b, c, s)
            boxes.append(b)
            classes.append(c)
            scores.append(s)

        boxes = np.concatenate(boxes)
        boxes = self._roi_convert_to_diagonal(boxes)
        classes = np.concatenate(classes)
        scores = np.concatenate(scores)

        nboxes, nclasses, nscores = [], [], []
        for c in set(classes):
            inds = np.where(classes == c)
            b = boxes[inds]
            c = classes[inds]
            s = scores[inds]

            keep = self._nms_boxes(b, s)

            nboxes.append(b[keep])
            nclasses.append(c[keep])
            nscores.append(s[keep])

        if not nclasses and not nscores:
            return None, None, None

        return np.concatenate(nboxes), np.concatenate(nclasses), np.concatenate(nscores)

    def _get_info(self, boxes, scores, classes):
        detections = []
        draw_info_list = []

        for box, score, cl in zip(boxes, scores, classes):
            top, left, right, bottom = box
            top = int(top)
            bottom = int(bottom * 480 / 640)
            left = int(left * 480 / 640)
            right = int(right)

            center = (int((right + top) // 2), int((bottom + left) // 2))
            boundary = (top, left, right, bottom)

            name = self.classes[cl]

            detections.append((name, score, center, boundary))
            draw_info_list.append((name, score, boundary, center))

        return detections, draw_info_list

    def _draw(self, image, draw_info_list):
        image = cv2.resize(image, (640, 480))

        for name, score, boundary, center in draw_info_list:
            top, left, right, bottom = boundary

            cv2.rectangle(image, (top, left), (int(right), int(bottom)), (255, 0, 0), 2)
            cv2.putText(image, '{0} {1:.2f}'.format(name, score),
                        (top, left - 6),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2)
            cv2.circle(image, center, 4, (0, 255, 0), 4)
            cv2.circle(image, (int(right), int(bottom)), 4, (255, 0, 0), 4)
            cv2.circle(image, (top, left), 4, (255, 0, 0), 4)

        return image

    def process(self, rknn_lite, frame):
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_rgb = cv2.resize(image_rgb, (self.image_size, self.image_size))

        outputs = rknn_lite.inference(inputs=[image_rgb])

        input0_data = outputs[0].reshape([3, -1] + list(outputs[0].shape[-2:]))
        input1_data = outputs[1].reshape([3, -1] + list(outputs[1].shape[-2:]))
        input2_data = outputs[2].reshape([3, -1] + list(outputs[2].shape[-2:]))

        input_data = list()
        input_data.append(np.transpose(input0_data, (2, 3, 0, 1)))
        input_data.append(np.transpose(input1_data, (2, 3, 0, 1)))
        input_data.append(np.transpose(input2_data, (2, 3, 0, 1)))

        boxes, classes, scores = self._yolov5_post_process(input_data)

        image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

        detections = []
        if boxes is not None:
            detections, draw_info_list = self._get_info(boxes, scores, classes)
            image = self._draw(image, draw_info_list)
        return image, detections
