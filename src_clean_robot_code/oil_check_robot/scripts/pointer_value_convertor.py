class PointerValueConvertor:
    def __init__(self, min_angle, max_angle, min_value, max_value):
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.min_value = min_value
        self.max_value = max_value

    def angle_to_value(self, angle):
        # 处理角度周期（确保在0-360）
        angle = angle % 360

        # 计算总有效角度范围（顺时针从min_angle到max_angle）
        if self.min_angle > self.max_angle:
            total_range = (360 - self.min_angle) + self.max_angle
        else:
            total_range = self.max_angle - self.min_angle

        # 计算当前角度相对于min_angle的偏移量（顺时针）
        if angle >= self.min_angle:
            offset =  angle - self.min_angle
        else:
            offset = (360 - self.min_angle) + angle

        # 线性映射到数值
        value = (offset / total_range) * self.max_value

        # 确保数值在min_value到max_value之间
        value =  max(self.min_value, min(value, self.max_value))

        return round(value, 2)  # 保留两位小数
