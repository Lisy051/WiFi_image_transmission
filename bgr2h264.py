import cv2
import numpy
import subprocess
import mss  #监听屏幕(比PIL快一些)
sizeStr = '1920x1080'
out = 'udp://100.100.100.241:5600'   #使用标准输入输出时填'-'；使用udp时，这里填'udp://127.0.0.1:6666'
#out = './480p.mp4'
#out = 'rtp://192.168.0.101:5600'
command = ['ffmpeg',
    #'-y', '-an',
    #'-hwaccel', 'vaapi',
    '-f', 'rawvideo',
    '-vcodec','rawvideo',
    '-pix_fmt', 'bgr24',
    '-s', sizeStr,
    '-r', '30',
    '-i', '-',
    '-c:v', 'libx264',
    '-pix_fmt', 'yuv420p',
    '-preset', 'ultrafast',
    '-tune', 'zerolatency',
    #'-max_delay', '100',
    '-f',
    #'rtp',
    'h264',
    #'-rtsp_transport', 'tcp',
    #'-g', '5', 
    #'-vb', '1200k',
    out]

# ffmpeg -f rawvideo -vcodec rawvideo -pix_fmt bgr24 -s 1920x1080 -r 30 -i - -c:v libx264 -pix_fmt yuv420p -preset
pipe = subprocess.Popen(command, shell=False, stdin=subprocess.PIPE)
#cam = cv2.VideoCapture(0)
#print(cam.isOpened())
#cam.set(cv2.CAP_PROP_FOCUS,cv2.VideoWriter_fourcc(*"MJPG"))
#cam.set(cv2.CAP_PROP_FRAME_WIDTH,1280)    #宽
#cam.set(cv2.CAP_PROP_FRAME_HEIGHT,720)   #高 
#cam.set(cv2.CAP_PROP_FPS,30)            #FPS 30~120
#print(cam.get(3))
#ret,img = cam.read()
mon = {"top": 0, "left": 0, "width": 1920, "height": 1080}
sct = mss.mss()
while True:
    #ret,img = cam.read()
    #img = cv2.resize(img,(1920,1080))
    #cv2.imshow('img',img)
    #cv2.waitKey(1)
    img = sct.grab(mon)    #捕获画面
    img = cv2.cvtColor(numpy.array(img),cv2.COLOR_BGRA2BGR)#色彩空间转换
    pipe.stdin.write(img.tobytes())
    #data = pipe.stdout.read()
