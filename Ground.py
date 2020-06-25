import cv2
import numpy as np
import numpy
import os,sys,time,traceback
import threading
import socket
import netifaces#这个模块好像在Linux下有问题
from pywifi import const, PyWiFi, Profile

def Run():
    wifi = WiFi()
    print('正在尝试连接WiFi')
    isok = False  
    while not isok:
        isok = wifi.connect_wifi(wifi.name, wifi.key)
    udp = GroundUDP()
    udp.create_link()
    print('正在尝试连接到图传')
    udp.GroundReadMessage()
        

class WiFi(object):
    name = r"ChinaNet-fMrj"
    key = r"gzc7riuq"
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

        self.iface.remove_all_network_profiles()                    # 删除其他配置
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

class GroundUDP:
    AirAddr = '127.0.0.1'
    link = False
    def __init__(self):
        self.GroundIP()

    #-------------------------#
    #创建UDP
    #-------------------------#
    def createUDP(self,IP='127.0.0.1'):
        self.s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        print('正在尝试绑定本机IP')
        try:
            print(IP)
            self.s.bind((IP,8080))
            print('绑定成功')
        except:
            print('绑定失败')
            exit(1)

    #-------------------------#
    #获取本机IP，并绑定到UPD
    #-------------------------#
    def GroundIP(self):
        routingNicName = netifaces.gateways()['default'][netifaces.AF_INET][1]   #网络适配器信息
        for interface in netifaces.interfaces():
            if interface == routingNicName:
                routingIPAddr = netifaces.ifaddresses(interface)[netifaces.AF_INET][0]['addr']   #获取IP

        self.createUDP(IP = routingIPAddr)

    #-------------------------#
    #发送WiFi信号到天空端
    #-------------------------#
    def GroundSendMessage(self,data):
        try:
            self.s.sendto(data.encode('utf-8'),(self.AirAddr,8080))
            #print(data)
        except:print('发送失败')

    #-------------------------#
    #监听UPD信息
    #-------------------------#
    def GroundReadMessage(self):
        print('UDP监听线程已打开')
        i=0
        while True:
            #print('img')
            t = time.time()
            data,addr = self.s.recvfrom(65535)
            t = time.time() - t
            if t>0 and t<1:
                fps = 1/t
            else:
                fps = 1
            i += 1
            if i == 10:
                self.GroundSendMessage(str(fps))
                i = 0
            imgData = numpy.frombuffer(data,numpy.uint8)
            img = cv2.imdecode(imgData,cv2.IMREAD_COLOR)
            cv2.imshow('Air',img)
            cv2.waitKey(1)

        print('UDP监听线程已关闭')

    #-------------------------#
    #发送连接信号到天空端
    #-------------------------#
    def create_link(self):
        #self.AirAddr = netifaces.gateways()['default'][netifaces.AF_INET][0]   #网关
        self.AirAddr = '192.168.1.3'
        while True:
            print('正在尝试连接')
            try:
                self.GroundSendMessage('Hello Air!')
                print(self.s.recvfrom(65535))
                break
            except:
                self.AirAddr = input('请手动输入图传的IP:')
            time.sleep(1)
        print('连接成功')

if __name__ == "__main__":
    Run()
