#!/usr/bin/env python3

import matplotlib.pyplot as plt
import math

#设计一个节点结构体，包含位置x，y，搜索父节点，代价，id（xy计算的结果）
class Node:
        def __init__(self,x,y,parent,cost,index):
                self.x = x
                self.y = y
                self.parent = parent
                self.cost = cost
                self.index = index

#沿着封闭点集合从目标点一直找到起始点
def calc_path(goaln,closeset):
        rx,ry=[goaln.x],[goaln.y]
        print (closeset[-1])
        parentn = closeset[-1]
        while parentn!=None:
                rx.append(parentn.x)
                ry.append(parentn.y)
                parentn = parentn.parent
        return rx,ry

#astar路径生成函数
def astar_plan(sx,sy,gx,gy):
    #生成障碍物地图
        ox,oy,xwidth,ywidth=map_generation()
        plt.figure('Astar algorithm demo')
        plt.plot(ox,oy,'ks')
        plt.plot(sx,sy,'bs')
        plt.plot(gx,gy,'ro')
        motion = motion_model()
    #初始化开放节点列表和闭合节点列表
        openset,closeset=dict(),dict()
    #初始化起点和终点两个节点
        sidx = sy*xwidth+sx
        gidx = gy*xwidth+gx
        starn=Node(sx,sy,None,0,sidx)
        goaln=Node(gx,gy,None,0,gidx)
    #初始节点塞入开放节点列表
        openset[sidx] = starn
        while 1:
        #找到开放列表中最低成本的节点，开放节点代价加上该节点到目标节点的欧几里得距离作为最终代价
                c_id = min(openset,key=lambda o:openset[o].cost + h_cost(openset[o],goaln))
                curnode = openset[c_id]
        #如果该节点恰好是目标节点，则直接打印到达，跳出循环
                if curnode.x == goaln.x and curnode.y == goaln.y:
                        print ('find goal')
                        closeset[-1] = curnode
                        break
        #如果当前节点不是目标节点，则将其添加到关闭列表中，并将其从开放列表中删除。
                else:
                        closeset[c_id] = curnode
                        plt.plot(curnode.x,curnode.y,'gx')
                        if len(openset.keys())%10==0:
                                plt.pause(0.01)
                        del openset[c_id]
                #对于每种可能的移动方式（由motion定义）生成一个新的节点newnode。
                for j in range(len(motion)):
                        newnode = Node(curnode.x+motion[j][0],
                                                   curnode.y+motion[j][1],
                                                   curnode,
                                                   curnode.cost + motion[j][2],
                                                   c_id)
                        n_id = index_calc(newnode,xwidth)
                        #如果新节点已经在关闭列表中，则跳过。
                        if n_id in closeset:
                                continue
                        #如果新节点在地图边界之外或不可达，也跳过。
                        if node_verify(newnode,ox,oy):
                                continue
                        #如果新节点不再开放列表中，将其添加到开放列表中
                        if n_id not in openset:
                                openset[n_id] = newnode
            #如果新节点已经在开放列表中，但是通过当前的移动方式得到的新节点的成本更低，则更新该节点。
                        else:
                                if openset[n_id].cost >= newnode.cost:
                                        openset[n_id] = newnode
        px,py = calc_path(goaln,closeset)#使用calc_path(goaln,closeset)函数计算从目标节点到起始节点的路径，并返回该路径。
        return px,py

#栅格地图建模生成函数，将所有被占据的点放入一个数组中，搜索到这个点时，遍历障碍栅格点即可确认是否可达
def map_generation():
        ox,oy=[],[]
        for i in range(60):
                ox.append(i)
                oy.append(0)

        for i in range(60):
                ox.append(i)
                oy.append(60)

        for i in range(60):
                ox.append(0)
                oy.append(i)

        for i in range(60):
                ox.append(60)
                oy.append(i)

        for i in range(25):
                ox.append(i)
                oy.append(20)

        for i in range(40):
                ox.append(35)
                oy.append(i)

        for i in range(40):
                ox.append(50)
                oy.append(60-i)

        minx = min(ox)
        miny = min(oy)
        maxx = max(ox)
        maxy = max(oy)
        xwidth = maxx-minx
        ywidth = maxy-miny

        return ox,oy,xwidth,ywidth

#运动模型函数，每次运动可以到达周围的八个栅格，上下左右栅格距离是1，协方向栅格距离是根号2
def motion_model():
        motion =[[1,0,1],
                         [1,1,math.sqrt(2)],
                         [1,-1,math.sqrt(2)],
                         [0,1,1],
                         [0,-1,1],
                         [-1,1,math.sqrt(2)],
                         [-1,0,1],
                         [-1,-1,math.sqrt(2)]]
        return motion

#启发函数计算，计算出当前点到最终目标点的欧几里得距离
def h_cost(node,goal):
        w = 1.0
        h = w*(abs(goal.x-node.x) + abs(goal.y-node.y))
        return h

#计算节点id，其实就是y*整体地图宽度+x，可类比二维数组id
def index_calc(node,xwid):
        n_id = node.y*xwid + node.x
        return n_id

#判断当前节点是否在障碍物栅格集合内，来判断该点是否可达
def node_verify(node,ox,oy):
        if (node.x,node.y) in zip(ox,oy):
                return True
        else:
                return False

def main():
        sx,sy=15,15 #出发点坐标
        gx,gy=55,50 #终点坐标
        rx,ry=        astar_plan(sx,sy,gx,gy) #实现A*
        print (rx,ry)
        plt.plot(rx,ry,'r-',linewidth=3) #绘制A*路径
        plt.show()

if __name__ =="__main__":
        main()
