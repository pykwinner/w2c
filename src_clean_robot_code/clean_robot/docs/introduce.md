# 创意赛服务机器人竞技A赛项

## 业务流程

1. 运行到（0.5，0.0）面朝y轴负方向位置；
2. apriltag_ros扫描二维码，确定要清扫区域位置；
3. movebase导航到第一个区域；
4. 生成弓字型路径，coremove执行清扫；
5. movebase导航到第二个区域；
6. 生成弓字型路径，coremove执行清扫；
7. movebase导航到第三个区域；
8. 生成弓字型路径，coremove执行清扫；
9. 返回出发区域；
10. 任务结束；

## 示例代码描述

- path_generate：在一块矩形区域中，生成弓字型路径全覆盖示例；
- three_path_generate：在比赛地图上三块矩形区域中，生成弓字型路径全覆盖示例；
- movebase_client：航点调度示例，到达每块清扫区域起点，返回出发点的示例；
- coremove_client：弓字型清扫示例，先导航到一个起始点，然后调用coremove，完成路径清扫；
- apriltag_detect: 导航到某一个点，识别apriltag确定清扫区域，id为1去往某个点，id为2去到另一个点；