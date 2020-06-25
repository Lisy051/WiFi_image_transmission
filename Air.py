import socket
import serial
import serial.tools.list_ports
import binascii
import threading
import cv2
import numpy,numpy as np
import math
import time
import sys

UART_LOCK = threading.Lock()
imge_LOCK = threading.Lock()
UDP_LOCK = threading.Lock()
delay_LOCK = threading.Lock()

#==================================================
#调用
#==================================================
def Run():
    global imgL
    global imgR
    global delay
    global osd_img

    fps = 30
    delay = 33

    uart = UART_Thread()
    osd = OSD()
    sock = Air()

    piCam = cv2.VideoCapture(0)

    if not piCam.isOpened():
        print('没有找到树莓派摄像头')
        return

    piCam.set(6,cv2.VideoWriter_fourcc(*"MJPG"))
    piCam.set(3,1280)
    piCam.set(4,720)

    print('摄像头已打开')
    ret,imgL = piCam.read()
    ret,imgR = usbCam.read()
    osd_img = osd.run(imgL)

    uart.Uart_select()#测试
    #uart.Pi_UART()#实际运行
    uartThead = threading.Thread(target=uart.read)
    uartThead.start()
    uartData = {}

    wifi = WiFi()
    print('正在尝试连接WiFi')
    isok = False  
    while not isok:
        isok = wifi.connect_wifi(wifi.name, wifi.key)

    sock.link()
    time.sleep(1)
    udp_RX_Thend = threading.Thread(target=sock.udp_ReadMessage)
    udp_RX_Thend.start()
    udp_TX_Thead = threading.Thread(target=sock.udp_SendImge)
    udp_TX_Thead.start()
	
    time.sleep(1)

    print('准备就绪')
    while True:
        
        imge_LOCK.acquire()
        ret,imgL = piCam.read()
        ret,imgR = usbCam.read()
        osd_img = osd.run(imgL)
        imge_LOCK.release()

        cv2.imshow('osd',osd_img)

        temp = udp.getData()
        if temp < fps-5:#帧数不够
            fps = temp
        else:
            fps += 5
            if fps >33:
                fps =33
        print(fps)
        delay_LOCK.acquire()
        delay = int(1000/fps)
        delay_LOCK.release()
        osd.wifi = int(fps/5)

        uartData = uart.getdata()
        osd.roll = uartData[0]       #横滚角0-360
        osd.pitch = uartData[1]      #俯仰角0-360
        osd.direction = uartData[2]  #指南针方向0-360
        osd.longitude = uartData[3]  #经度0-360
        osd.latitude = uartData[4]   #纬度0-360
        osd.altitude = uartData[5]   #飞行高度
        osd.Voltage = uartData[6]    #电压
        self.Battery = uartData[7]   #电量0-1.00
        osd.power = uartData[8]      #油门0-1.00
        osd.speed_D = uartData[9]    #速度xy
        osd.speed_H = uartData[10]   #速度z
        osd.home = uartData[11]      #返航方向
        osd.distance = uartData[12]  #飞行距离

        cv2.waitKey(1)

#==================================================
#串口通信的实现
#==================================================
class UART_Thread:

    ser = serial.Serial()
    data = {}
    data[0] = 0
    data[1] = 0
    data[2] = 0
    data[3] = 0
    data[4] = 0
    data[5] = 0
    data[6] = 0
    data[7] = 0
    data[8] = 0
    data[9] = 0
    data[10] = 0
    data[11] = 0
    data[12] = 0

    #-------------------------#
    #打开树莓派硬件串口
    #-------------------------#
    def Pi_UART(self):
        self.ser.port = '/dev/ttyAMA0'              #设置端口号
        self.ser.baudrate = 115200      #设置波特率
        self.ser.bytesize = 8           #设置数据位
        self.ser.stopbits = 1           #设置停止位
        self.ser.parity = "N"           #设置校验位
        self.ser.open()                 #打开串口,要找到对的串口号才会成功
        if(self.ser.isOpen()):
            print("串口打开成功")
        else:
            print("串口打开失败")

    #-------------------------#
    #关闭串口
    #-------------------------#
    def port_close(self):
        self.ser.close()
        if (self.ser.isOpen()):
            print("串口关闭失败")
        else:
            print("串口关闭成功")

    #-------------------------#
    #选择串口号
    #-------------------------#
    def Uart_select(self):
        #串口初始化
        port_list = list(serial.tools.list_ports.comports())
        k=0
        for i in port_list:
            print(i,k)
            k=k+1
        if len(port_list) <= 0:
            print("没有找到串口")
        else:
            serial_k=input("请选择要打开的串口:")
            while(int(serial_k) >= k or int(serial_k)<0):
                serial_k=input("错误！请重新选择要打开的串口:")
            k = int(serial_k)
            serial_list = list(port_list[k])
            serialName = serial_list[0]
            print(serialName)
            self.ser.port = serialName      #设置端口号
            self.ser.baudrate = 115200      #设置波特率
            self.ser.bytesize = 8           #设置数据位
            self.ser.stopbits = 1           #设置停止位
            self.ser.parity = "N"           #设置校验位
            self.ser.open()                 #打开串口,要找到对的串口号才会成功
            if(self.ser.isOpen()):
                print("串口打开成功")
            else:
                print("串口打开失败")

    #-------------------------#
    #发送数据
    #-------------------------#
    def send(self,send_data):
        if (self.ser.isOpen()):
            self.ser.write(send_data.encode('gbk'))  #gbk 编码发送
            print("发送成功：",send_data)
        else:
            print("串口未打开，发送失败")

    #-------------------------#
    #串口监听
    #-------------------------#
    def read(self):
        print('串口监听线程已开启！')
        num = 0
        tempData ={}
        while True:
            if not (self.ser.isOpen()):
                print('串口未打开')
                break
            
            # 获得接收缓冲区字符
            count = self.ser.inWaiting()
            if count != 0:
                # 读取内容
                recv = self.ser.read(count).encode('gbk')
                if recv == 'over':
                    if num == 13:
                        UART_LOCK.acquire()
                        self.data = tempData
                        UART_LOCK.release()
                    num = 0
                else:
                    try:
                        tempData[num] = float(recv)
                    except:pass
                num += 1
                if num > 13:num=0

            # 清空接收缓冲区
            self.ser.flushInput()
            # 必要的软件延时
            time.sleep(0.03)

        print('串口监听线程已关闭！')

    #-------------------------#
    #读取数据
    #-------------------------#
    def getdata(self):
        with UART_LOCK:
            return self.data

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

#==================================================
#无线传输的实现
#==================================================
class Air:

    data = 30
    GroudAddr = '192.168.12.1'
    def __init__(self):
        self.AirIP()

    #-------------------------#
    #打开UDP通信
    #-------------------------#
    def createUDP(self,IP='127.0.0.1'):
        self.s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        #self.s1 = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        print('正在尝试绑定本机IP')
        try:
            print(IP)
            self.s.bind((IP,8080))
            #self.s1.bind((IP,8081))
            print('绑定成功')
        except:
            print('绑定失败')
            sys.exit(1)
    #-------------------------#
    #绑定本机IP地址
    #-------------------------#
    def link(self):
        self.createUDP(IP='192.168.12.100')

    #-------------------------#
    #发送图像信息
    #-------------------------#
    def udp_SendImge(self):
        global osd_img
        global delay
        print('图像传输线程已打开')
        while True:
            num = 65
            with delay_LOCK:
                d = delay
            while True:
		        #压缩参数，后面cv2.imencode将会用到，对于jpeg来说，15代表图像质量，越高代表图像质量越好为 0-100，默认65
                encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),num]
                with imge_LOCK:
                    result, imgencode = cv2.imencode('.jpg', osd_img, encode_param)
                l = len(imgencode)
                if l > 50000:
                    num = num-10
                    if num <0:break
                else:break
            data = numpy.array(imgencode)
            #将numpy矩阵转换成字符形式，以便在网络中传输
            stringData = data.tostring()
            self.s.sendto(stringData,(self.GroudAddr,8080))
            cv2.waitKey(d)

    #-------------------------#
    #读取地面站数据
    #-------------------------#
    def udp_ReadMessage(self):
        while True:
            data = self.s.recvfrom(1024)
            try:
                data = float(data)
                UDP_LOCK.acquire()
                self.data = data
                UDP_LOCK.release()
            except:pass
            print('udpGetData:%.2f' % data)

    def getData(self):
        with UDP_LOCK:
            return self.data

#==================================================
#自动连接WiFi的实现
#==================================================
class WiFi(object):
    name = r"mySSID"
    key = r"12345678"
    # 创建对象自动初始化，类似Java的构造函数
    def __init__(self):
        wifi = PyWiFi()                     # 创建一个无线对象
        self.iface = wifi.interfaces()[0]   # 获取当前机器第一个无线网卡

    # 查看wifi的连接状态
    def wifi_connect_status(self):
        """
        判断本机是否有无线网卡，以及连接状态
        :return:已连接或存在网卡返回1，否则返回0
        """
        ret_list = []
        # 判断是否连接成功
        if self.iface.status() in \
                [const.IFACE_CONNECTED, const.IFACE_CONNECTING, const.IFACE_INACTIVE]:
            return self.iface.name()        # 连接成功显示连接设备
        else:
            return "not connected!"        # 连接失败返回失败信息

    """
    扫描附近wifi
        乱码问题：
        把wifi_info.ssid重新编码为gb18030
        wifi_info.ssid.encode('raw_unicode_escape','strict').decode('gb18030')
        我也不清楚他为什么不全用unicode
        ssid出来应该是unicode  但是  你往profile里面写的时候  必须是gb18030
        所以这么一个操作
        你会发现gb18030在控制台和py的某些控件上输出是乱码  是因为 控制台是utf-8
        想在这上面输出中文的话你得encode('raw_unicode_escape','strict').decode()
    """
    def scan_wifi(self, scantime=5):
        """
        :param scantime:    指定扫描时间，默认扫描时间为5秒
        :return:            返回的是一个network dictionary,key=bssid,value=ssid
        """
        self.iface.scan()                                           # 扫描附近wifi
        time.sleep(scantime)
        basewifi = self.iface.scan_results()
        dict = {}
        for i in basewifi:
            dict[i.bssid] = \
                i.ssid.encode(encoding='raw_unicode_escape', errors='strict').decode()
            
        return dict

    def Signal(self, scantime=5):
        """
        :param scantime:    指定扫描时间，默认扫描时间为5秒
        :return:            返回的是一个network dictionary,key=bssid,value=ssid
        """
        self.iface.scan()                                           # 扫描附近wifi
        time.sleep(scantime)
        basewifi = self.iface.scan_results()
        s = {}
        num = 0
        x = 0
        for i in basewifi:
            if i.ssid == self.name:
                s[num] = i.signal
                num += 1
        for i in range(0,num):
            x += s[i]
            
        return int(x/num)
    #-------------------------#
    #连接到指定WiFi
    #-------------------------#
    def connect_wifi(self, wifi_ssid, password):
        profile = Profile()                                         # 配置文件
        profile.ssid = wifi_ssid                                    # wifi名称
        profile.auth = const.AUTH_ALG_OPEN                          # 需要密码
        profile.akm.append(const.AKM_TYPE_WPA2PSK)                  # 加密类型
        profile.cipher = const.CIPHER_TYPE_CCMP                     # 加密单元
        profile.key = password                                      # wifi密码

        #self.iface.remove_all_network_profiles()                    # 删除其他配置
        tmp_profile = self.iface.add_network_profile(profile)       # 加载配置

        self.iface.connect(tmp_profile)                             # link start
        time.sleep(10)                                              # 尝试10s是否成功
        isok = True
        if self.iface.status() == const.IFACE_CONNECTED:
            return isok                                             # 连接成功
        else:
            isok = False                                            # 连接失败设置isok = False
        self.iface.disconnect()                                     # 避免超时后连接成功手动断开一下，因为在一定时间内连接失败用户会继续重试连接
        time.sleep(1)
        return isok



Run()
