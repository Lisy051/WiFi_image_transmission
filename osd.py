import cv2
import numpy
import numpy as np

import math

#==================================================
#图像叠加的实现
#==================================================
class OSD:

    def __init__(self):
        self.roll = 0       #横滚角0-360
        self.pitch = 0      #俯仰角0-360
        self.direction = 0  #指南针方向0-360
        self.longitude = 0  #经度0-360
        self.latitude = 0   #纬度0-360
        self.altitude = 0   #飞行高度
        self.Voltage = 12.6 #电压
        self.Battery = 1    #电量0-1.00
        self.power = 0      #油门0-1.00
        self.speed_D = 0    #速度xy
        self.speed_H = 0    #速度z
        self.wifi = 0       #WIFI
        self.home = 0       #返航方向
        self.distance = 0   #飞行距离

    #-------------------------#
    #OSD运行一次
    #-------------------------#
    def run(self,imge):

        self.size = imge.shape
        self.imge = imge.copy()

        self.Gyroscope(self.roll,self.pitch)#水平仪(横滚角 and 俯仰角)
        self.Compass(self.direction,self.home)#电子罗盘电子罗盘(指南针方向and 返航方向)
        self.GPS(self.longitude,self.latitude)#经纬度
        self.Altimeter(self.altitude,self.distance)#高度计(飞行高度)
        self.Voltmeter(self.Voltage,self.Battery)#电压表(电压和电量)
        self.Speedometer(self.power,self.speed_D,self.speed_H)#速度计(油门和速度)
        self.Signal(self.wifi)#信号(WIFI)
        
        return self.imge

    #-------------------------#
    #水平仪(横滚角 and 俯仰角)
    #-------------------------#
    def Gyroscope(self,roll,pitch):
        
        #中心坐标
        origin_x = int(self.size[1]/2)
        origin_y = int(self.size[0]/2)
        
        #水平仪(L)
        x0 = origin_x - int(30*math.cos(roll*math.pi/180))
        y0 = origin_y - int(30*math.sin(roll*math.pi/180)) - int(origin_y/2*math.sin(pitch*math.pi/180))
        x1 = x0 - int(self.size[1]/10*math.cos(roll*math.pi/180))
        y1 = y0 - int(self.size[1]/10*math.sin(roll*math.pi/180))
        #水平仪(R)
        x2 = origin_x + int(30*math.cos(roll*math.pi/180))
        y2 = origin_y + int(30*math.sin(roll*math.pi/180))  - int(origin_y/2*math.sin(pitch*math.pi/180))
        x3 = x2 + int(self.size[1]/10*math.cos(roll*math.pi/180))
        y3 = y2 + int(self.size[1]/10*math.sin(roll*math.pi/180))
        #水平仪(origin)
        x4 = origin_x - int(30*math.sin(-roll*math.pi/180))
        y4 = origin_y - int(30*math.cos(-roll*math.pi/180))
        #横滚角文字位置text
        x5 = origin_x - int(60*math.sin(-roll*math.pi/180))
        y5 = origin_y - int(60*math.cos(-roll*math.pi/180))
        #俯仰角文字位置text
        x6 = x2 + int(self.size[1]/8*math.cos(roll*math.pi/180))
        y6 = y2 + int(self.size[1]/8*math.sin(roll*math.pi/180))

        cv2.circle(self.imge,(origin_x,origin_y),10,(0,100,0),3)                      #在画面的中心标记个圆圈圈
        cv2.line(self.imge,(origin_x,origin_y-10),(origin_x,origin_y-20),(0,100,0),3) #画出表示无人机方向的线
        cv2.line(self.imge,(origin_x,origin_y),(x4,y4),(0,100,0),3) #画出表示无人机方向的线
        cv2.line(self.imge,(x0,y0),(x1,y1),(0,100,0),3)#画出水平仪(L)的线
        cv2.line(self.imge,(x2,y2),(x3,y3),(0,100,0),3)#画出水平仪(R)的线

        cv2.putText(self.imge,'Roll%.1f'% abs(roll),(x5,y5),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,100,0),2)#横滚角
        cv2.putText(self.imge,'Pitch%.1f'% abs(pitch),(x6,y6),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,100,0),2)#俯仰角

    #-------------------------#
    #电压表(电压和电量)
    #-------------------------#
    def Voltmeter(self,Voltage,Battery):

        #颜色与电量相关
        if Battery > 0.75:
            colour = (0,255,0)#绿色
        elif Battery > 0.4:
            colour = (0,255,155)#黄绿色
        elif Battery > 0.2:
            colour = (0,0,255)#红色
        else:
            colour = (0,0,100)#深红色

        x0 = int(self.size[1]/100)
        y0 = int(self.size[0]*28/30)
        x1 = int(self.size[1]/20)
        y1 = int(self.size[0]*29/30)
        x2 = int(x0+(x1-x0)*Battery)
        y2 = int(y0*1.01)
        x3 = int(x1*1.05)
        y3 = int(y1*0.99)

        Battery *=100

        cv2.rectangle(self.imge,(x0,y0),(x1,y1),(0,100,0),-1)#电池本体
        cv2.rectangle(self.imge,(x1,y2),(x3,y3),(0,100,0),-1)#电池头
        cv2.rectangle(self.imge,(x0,y0),(x2,y1),colour,-1)#电量

        cv2.putText(self.imge,'Battery:%.1f%%'% Battery,(x0,y1+15),cv2.FONT_HERSHEY_SIMPLEX,0.5,colour,2)#剩余电量百分比
        cv2.putText(self.imge,'%.1fV'% Voltage,(x3+5,y3+3),cv2.FONT_HERSHEY_SIMPLEX,0.5,colour,2)#电压

    #-------------------------#
    #电子罗盘(指南针方向and 返航方向)
    #-------------------------#
    def Compass(self,direction,home):

        wight = self.size[1]

        y0 = int(self.size[0]/25)
        y1 = y0 +15
        y2 = y0 -10
        y3 = y1 +10
        x0 = int(wight/2)
        x1 = int((home+direction)%360/360*wight)
        x = {}
        
        for i in range(0,36):
            x[i] = int((i*10+direction)%360/360*wight)
            cv2.line(self.imge,(x[i],y0),(x[i],y1),(0,100,0),3)
            if i == 0:
                cv2.putText(self.imge,'S',(x[i]-5,y1+15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,100,0),2)#指南针角度
            elif i == 9:
                cv2.putText(self.imge,'W',(x[i]-5,y1+15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,100,0),2)#指南针角度
            elif i == 18:
                cv2.putText(self.imge,'N',(x[i]-5,y1+15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,100,0),2)#指南针角度
            elif i == 27:
                cv2.putText(self.imge,'E',(x[i]-5,y1+15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,100,0),2)#指南针角度

        cv2.line(self.imge,(x1,y0-5),(x1,y1+5),(0,255,0),2)#返航方向
        cv2.putText(self.imge,'H',(x1-5,y1+15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)#返航方向
        cv2.line(self.imge,(x0,y2),(x0,y3),(0,0,150),2)
        cv2.putText(self.imge,'%.1f'% abs(direction),(x0-15,y3+20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,100,0),2)#指南针角度

    #-------------------------#
    #速度计(油门和速度)
    #-------------------------#
    def Speedometer(self,power,speed_D,speed_H):

        x = int(self.size[1]*49/50)
        y = self.size[0]
        r = int(self.size[1]/20)
        x0 ={}
        y0 ={}
        x1 ={}
        y1 ={}
        x2 = x - int(30*math.cos(power*math.pi/200))
        y2 = y - int(30*math.sin(power*math.pi/200))
        x3 = x2 - int((r-15)*math.cos(power*math.pi/200))
        y3 = y2 - int((r-15)*math.sin(power*math.pi/200))
        x4 = int(self.size[1]*17/20)
        y4 = int(self.size[0]*29/30)
        for i in range(0,36):
            x0[i] = x - int(r*math.cos(i*math.pi/18))
            y0[i] = y - int(r*math.sin(i*math.pi/18))
            x1[i] = x0[i] - int(15*math.cos(i*math.pi/18))
            y1[i] = y0[i] - int(15*math.sin(i*math.pi/18))
            cv2.line(self.imge,(x0[i],y0[i]),(x1[i],y1[i]),(0,100,0),2)

        cv2.line(self.imge,(x2,y2),(x3,y3),(0,0,255),3)
        cv2.circle(self.imge,(x,y),r,(0,100,0),3)                      #在画面的右下角标记个圆圈圈
        cv2.circle(self.imge,(x,y),r+15,(0,100,0),3)                      #在画面的右下角标记个圆圈圈
        cv2.putText(self.imge,'%.1f%%'% power,(x-25,self.size[0]-5),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,100,0),2)
        cv2.putText(self.imge,'D:%.1fm/s'% speed_D,(x4-30,y4),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,100,0),2)
        cv2.putText(self.imge,'H:%.1fm/s'% speed_H,(x4-30,y4+15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,100,0),2)

    #-------------------------#
    #高度计(飞行高度)
    #-------------------------#
    def Altimeter(self,altitude,distance):

        x0 = int(self.size[1]*38/40)
        y0 = int(self.size[0]*26/30)
        cv2.putText(self.imge,'H:%.1fm'% altitude,(x0-30,y0),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,100,0),2)
        cv2.putText(self.imge,'D:%.1fm'% altitude,(x0-30,y0-15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,100,0),2)

    #-------------------------#
    #信号(WIFI)
    #-------------------------#
    def Signal(self,signal):

        if signal >= 5:
            colour = (0,255,0)
        elif signal >= 4:
            colour = (0,155,70)
        elif signal >= 3:
            colour = (0,255,200)
        elif signal >= 2:
            colour = (0,0,255)
        elif signal >= 1:
            colour = (0,0,150)
        else:
            colour = (0,0,30)

        x0 = int(self.size[1]/100) +55
        y0 = int(self.size[0]*28/30)-10
        x ={}
        y ={}
        for i in range(0,signal):
            x[i] = int(self.size[1]/100) +10*i
            y[i] = int(self.size[0]*28/30)-10 - 5*i
            cv2.line(self.imge,(x[i],y0),(x[i],y[i]),colour,3)
        #cv2.putText(self.imge,'%ddbm'% signal,(x0,y0),cv2.FONT_HERSHEY_SIMPLEX,0.5,colour,2)

    #-------------------------#
    #经纬度
    #-------------------------#
    def GPS(self,longitude,latitude):
        x0 = int(self.size[1]*8/9)
        y0 = int(self.size[0]/25)+60
        cv2.putText(self.imge,'E %.2f'% longitude,(x0,y0),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,100,0),2)
        cv2.putText(self.imge,'N %.2f'% latitude,(x0,y0+15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,100,0),2)

