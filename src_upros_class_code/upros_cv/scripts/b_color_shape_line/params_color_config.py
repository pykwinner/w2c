#!/usr/bin/env python3  

from dynamic_reconfigure.server import Server
from upros_cv.cfg import params_colorConfig
import rospy 

# 定义回调函数，处理参数更新的请求
def callback(config, level):
    rospy.loginfo("Reconfigure Request: %d %d %d %d %d %d",
                  config.HSV_H_MIN, 
                  config.HSV_S_MIN,
                  config.HSV_V_MIN,
                  config.HSV_H_MAX,
                  config.HSV_S_MAX,
                  config.HSV_V_MAX)
    return config

if __name__ == "__main__":
    rospy.init_node('params_color')
    # 创建动态配置服务器实例
    srv = Server(params_colorConfig, callback)
    rospy.loginfo("Spinning node")
    rospy.spin()