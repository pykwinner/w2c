#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import math

class Info():
        def __init__(self):
                # define robot move speed ,accelerate,radius ...and so on
                self.v_min = -0.5
                self.v_max = 3.0
                self.w_max = 50.0 * math.pi / 180.0
                self.w_min = -50.0 * math.pi / 180.0
                self.vacc_max = 0.5
                self.wacc_max = 30.0 * math.pi / 180.0
                self.v_reso = 0.01
                self.w_reso = 0.1 * math.pi / 180.0
                self.radius = 1.0
                self.dt = 0.1
                self.predict_time = 4.0
                self.goal_factor = 1.0
                self.vel_factor = 1.0
                self.traj_factor = 1.0

def motion_model(x,u,dt):
        # robot motion model: x,y,theta,v,w
        x[0] += u[0] * dt * math.cos(x[2])
        x[1] += u[0] * dt * math.sin(x[2])
        x[2] += u[1] * dt
        x[3] = u[0]
        x[4] = u[1]
        return x

#生成速度窗口
def vw_generate(x,info):
        #从info中提取速度最大最小值
        Vinfo = [info.v_min,info.v_max,
                 info.w_min,info.w_max]
    #根据加速度和时间步长计算可能的最大速度移动范围
        Vmove = [x[3] - info.vacc_max * info.dt,
                 x[3] + info.vacc_max * info.dt,
                         x[4] - info.wacc_max * info.dt,
                         x[4] + info.wacc_max * info.dt]
    #创建一个新的列表vw，用于存储速度和角速度的上下界
        vw = [max(Vinfo[0],Vmove[0]),min(Vinfo[1],Vmove[1]),
              max(Vinfo[2],Vmove[2]),min(Vinfo[3],Vmove[3])]
        return vw

def traj_cauculate(x,u,info):
        ctraj = np.array(x)
        xnew = np.array(x) # Caution!!! Don't use like this: xnew = x, it will change x value when run motion_modle below
        time = 0
    #在一定预测时间内，根据输入预测后续轨迹
        while time <= info.predict_time:
        #根据运动模型生成新点
                xnew = motion_model(xnew,u,info.dt)
        #新点加入局部轨迹中
                ctraj = np.vstack((ctraj,xnew))
                time += info.dt
        return ctraj

#dwa算法核心
def dwa_core(x,u,goal,info,obstacles):
        
        vw = vw_generate(x,info)
        best_ctraj = np.array(x)
        min_score = 10000.0
    
        for v in np.arange(vw[0],vw[1],info.v_reso):
                for w in np.arange(vw[2],vw[3],info.w_reso):
                        # 在指定速度/角速度分辨率下遍历不同的可能路径
                        ctraj = traj_cauculate(x,[v,w],info)
                        # 计算路径三个得分总和
                        goal_score = info.goal_factor * goal_evaluate(ctraj,goal)
                        vel_score = info.vel_factor * velocity_evaluate(ctraj,info)
                        traj_score = info.traj_factor * traj_evaluate(ctraj,obstacles,info)
                        ctraj_score = goal_score + vel_score + traj_score
                #计算当前轨迹，得分越小轨迹质量越高
                        if min_score >= ctraj_score:
                                min_score = ctraj_score 
                                u = np.array([v,w])
                                best_ctraj = ctraj

        return u,best_ctraj

def goal_evaluate(traj,goal):
        #通过计算路径数组中最后一个点与目标点之间的欧几里德距离来计算该路径的目标得分
        goal_score = math.sqrt((traj[-1,0]-goal[0])**2 + (traj[-1,1]-goal[1])**2)
        return goal_score

def velocity_evaluate(traj,info):
        #通过路径最后一个点的速度和最大速度的插值来计算得分
        vel_score = info.v_max - traj[-1,3]
        return vel_score

def traj_evaluate(traj,obstacles,info):
        #遍历路径中每一个点，通过计算该点到所有障碍物的最短距离来计算路径得分，如果最短距离小于机器人半径，则直接返回无穷大，得分为距离的倒数
        min_dis = float("Inf")
        for i in range(len(traj)):
                for ii in range(len(obstacles)):
                        current_dist = math.sqrt((traj[i,0] - obstacles[ii,0])**2 + (traj[i,1] - obstacles[ii,1])**2)

                        if current_dist <= info.radius:
                                return float("Inf")

                        if min_dis >= current_dist:
                                min_dis = current_dist
        
        return 1 / min_dis


#障碍物生成
def obstacles_generate():
#        Map shape and obstacles:
#         ___________________________________
#        |                                   |
#        |                                   |
#        |____________             |         |
#        |                         |         |
#        |                   goal  |         |
#        |                   O     |         |
#        |                         |         |
#        |                                   |
#        |       _____________               |
#        |                                   |
#        |     *(start)                      |
#        |                                   |                                      
#        |___________________________________|

        obstacles = np.array([[0,10],
                                                  [2,10],
                                                  [4,10],
                                                  [6,10],
                                                  [3,5],
                                                  [4,5],
                                                  [5,5],
                                                  [6,5],
                                                  [7,5],
                                                  [8,5],
                                                  [10,7],
                                                  [10,9],
                                                  [10,11],
                                                  [10,13]])
        return obstacles

def local_traj_display(x,goal,current_traj,obstacles):
        plt.cla()
    #画目标点
        plt.plot(goal[0],goal[1],'or',markersize=10)
    #画边框
        plt.plot([0,14],[0,0],'-k',linewidth=7)
        plt.plot([0,14],[14,14],'-k',linewidth=7)
        plt.plot([0,0],[0,14],'-k',linewidth=7)
        plt.plot([14,14],[0,14],'-k',linewidth=7)
    #画三条障碍物连线
        plt.plot([0,6],[10,10],'-y',linewidth=10)
        plt.plot([3,8],[5,5],'-y',linewidth=10)
        plt.plot([10,10],[7,13],'-y',linewidth=10)
    #画障碍物
        plt.plot(obstacles[:,0],obstacles[:,1],'*b',markersize=8)
    #画当前点位置
        plt.plot(x[0],x[1],'ob',markersize=10)
    #画当前点姿态
        plt.arrow(x[0],x[1],math.cos(x[2]),math.sin(x[2]),width=0.02,fc='red')
    #画局部轨迹
        plt.plot(current_traj[:,0],current_traj[:,1],'-g',linewidth=2)
        plt.grid(True)
        plt.pause(0.001)

def main():
    #起始点
        x = np.array([2,2,45*math.pi/180,0,0])
    
        u = np.array([0,0])
    
    #目标点
        goal = np.array([8,8])
 
        info = Info()
    
    #障碍物信息
        obstacles = obstacles_generate()
 
    #全局路径，先把起始点填入
        global_traj = np.array(x)
 
        plt.figure('DWA Algorithm')
 
        for i in range(2000):
        
                u,current_traj = dwa_core(x,u,goal,info,obstacles)
  
        #机器人运动模型
                x = motion_model(x,u,info.dt)
        
        #将当前点添加到全局路径中
                global_traj = np.vstack((global_traj,x))
  
        #展示局部路径
                local_traj_display(x,goal,current_traj,obstacles)
  
        #距离小于一定范围则可以判断到达，结束循环
                if math.sqrt((x[0]-goal[0])**2 + (x[1]-goal[1])**2) <= info.radius:
                        print ("Goal Arrived!")
                        break

        #绘制全局路径        
        plt.plot(global_traj[:,0],global_traj[:,1],'-r')
        plt.show()

if __name__ == "__main__":
        main()
