# WiFi_image_transmission

这是一个使用python3编写的WiFi图传程序，可以在windows和linux平台运行

需要安装以下python3模块：
opencv-python
numpy
pywifi
comtypes
pyserial
pyserial.tools

注意！
这个程序目前写得非常糟糕，仅供参考。
改进的方向：
1.在图像读取、压缩的那部分，可以使用多线程(threading.Thread)和队列(Queue.queue)来优化它的速度
2.图像压缩的那部分，可以考虑把图片分块发送，以此降低对CPU的压力
3.还可以直接调用ffmpeg读取摄像头，然后把读取到的数据发送到地面端再进行OSD界面的叠加
