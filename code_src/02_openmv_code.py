# 241014 同步到git仓库
# Title:openmv module
# openmv
#导入函数库
import sensor, image, time, math ,lcd
from pyb import Servo,UART,Timer   #用于控制舵机
from pid import PID
#开启时钟
clock = time.clock()# 跟踪FPS帧率
PValue=0.5
IValue=0
Angle_pid=PID(p=PValue,i=IValue)
kernel_size = 1 # 3x3==1, 5x5==2, 7x7==3, etc.
#初始卷积核
kerne2 = [-1, -1,  -1, \
          -1,  9,  -1, \
           -1,  -1,  -1]
#变量初始化
K=327           #测距比例常数 s=k*pix
OVSys_State=1#OpenMV系统状态量 1和3为循迹模式 2为避障模式
Distate=0

#通信数据包封装
Run     = bytearray([0x24,0x4F,0x4D,0x56,0x31,0x26,0x23])# $OMV1&#  直行
Left    = bytearray([0x24,0x4F,0x4D,0x56,0x32,0x26,0x23])# $OMV2&#  左转
Right   = bytearray([0x24,0x4F,0x4D,0x56,0x33,0x26,0x23])# $OMV3&#  右转
Stop    = bytearray([0x24,0x4F,0x4D,0x56,0x34,0x26,0x23])# $OMV4&#  停止

#阈值设置 根据实际情况进行更改
TRA_TH= [(128, 255)]                  #巡线的灰度值 阈值[(0, 64)][(128, 255)]
OBS_TH= [(20, 65, -4, 91, -1, 93)]   #避障的LAB值阈值 用于测距
Red_TH      = (46, 79, 28, 65, -24, 63)#红色
Blue_TH     = (24, 63, -12, 33, -69, -25)#蓝色
OBS_TH1      = [Red_TH,Blue_TH]
TRA_AngTH=30                   #巡线时角度阈值
OBS_DisTH=17                           #避障时距离阈值
Hor_Angle=0
Ver_Angle=0
#ROI区域设置
#(x,y,w,h,weight)=(矩形左上顶点的坐标(x,y),矩形宽度和高度(w,h),权重)
TRA_ROIS = [ # [ROI, weight]
        (0, 100, 160, 20, 0.7), # You'll need to tweak the weights for your app
        (0,  50, 160, 20, 0.3), # depending on how your robot is setup.
        (0,   0, 160, 20, 0.1)
       ]
Weight_Sum = 0                        #权值和初始化
for r in TRA_ROIS: Weight_Sum += r[4] #计算权值和
OBS_ROI = [(30 , 10, 100, 100)] #避障模式ROI区域

#Fun1:获得最大色块的位置索引函数
#输入:N个色块(blobs) 输出:N个色块中最大色块的索引(int i)
def Send_PWM(LP,RP):
    if LP <0 :
        PWML=abs(LP)+100
    else :
        PWML=LP
    if RP <0 :
        PWMR=abs(RP)+100
    else :
        PWMR=RP
    PWM= bytearray([0x50,0x57,0x4D,int(PWML),int(PWMR),0x24,0x23])# $OMV5&#  停止
    uart.write(PWM)
def Get_MaxIndex(blobs):
    maxb_index=0                      #最大色块索引初始化
    max_pixels=0                      #最大像素值初始化
    for i in range(len(blobs)):       #对N个色块进行N次遍历
        if blobs[i].pixels() > max_pixels:#当某个色块像素大于最大值
            max_pixels = blobs[i].pixels()#更新最大像素
            maxb_index = i                #更新最大索引
            return  maxb_index
#Function2:接收数据 接收数据并赋值给系统状态量
def Recive_Data():
    global OVSys_State
    if uart.any():                  #如果串口接收到数据
      OVSys_State=int(uart.read())  #将读到的数据强制转换为整数
      print(OVSys_State)            #用于串口通信调试

#舵机重定义与初始化
#水平舵机 0-180° 0°时正对摄像头正前方 左右各90° 左为正 右为负
#垂直舵机 0-180° 0°时正对摄像头正前方 上下各90° 下为正 上为负
Hor_servo=Servo(1)        #水平方向舵机
Ver_servo=Servo(2)        #垂直方向舵机
Hor_servo.angle(0)      #水平初始为-10 参数取值范围(-90,90)
Ver_servo.angle(-10)      #垂直初始为-10 参数取值范围(-90,90)
#TFT-LCD初始化
lcd.init()                #lcd函数初始化
lcd.set_direction(1)      #设置LCD显示方向 0和2是竖屏 1和3是横屏
#串口通信初始化
uart = UART(1,115200)     #串口1 波特率为115200 9600
uart.init(115200, bits=8, parity=None, stop=1)#字节长度为8 无校验位 停止位为 1
#摄像头初始化
sensor.reset()                     #初始化相机传感器
sensor.set_pixformat(sensor.RGB565)#设置相机模块的像素模式 16 bits/像素 GRAY为8
sensor.set_framesize(sensor.QQVGA) #设置相机模块的帧大小 160x120
sensor.skip_frames(30)             #跳过30帧 让相机图像在改变相机设置后稳定下来
sensor.set_auto_gain(False)        #关闭自动增益
sensor.set_auto_whitebal(False)    #关闭默认的白平衡
counter0=0
counter1=0
counter2=0
counter3=0
counter4=0
counter5=0
counter6=0
#主函数
while (1):
    clock.tick()
    if (OVSys_State==1)or (OVSys_State==3):#循迹模式或者循迹避障模式
            Hor_servo.angle(0)          #循迹模式舵机姿态调整
            Ver_servo.angle(90)           #水平-10 垂直向下70°
            sensor.set_pixformat(sensor.GRAYSCALE)#循迹模式 设置摄像头为灰度图
            TRA_img = sensor.snapshot().histeq()#截一帧图像 加强对比度好分割
            TRA_img.mean(1)
            TRA_img.binary([(0,35)])
            #在LCD上打印帧率和OpenMV工作模式
            TRA_img.draw_string(0, 60,"OpenMv\rMode:\rTracking\rMode")
            TRA_img.draw_string(0, 100, "FPS:\r%.2f"%(clock.fps()))
            #偏移角度计算
            Centroid_Sum = 0    #初始化质心和
            for r in TRA_ROIS:  #是ROI的元组
                #找到视野中ROI区域的色块,merge=true,将找到的图像区域合并
                blobs = TRA_img.find_blobs(TRA_TH, roi=r[0:4], pixels_threshold=100, area_threshold=100,merge=True)
                if blobs:#如果找到了多个色块 计算质心和
                    maxb_index = Get_MaxIndex(blobs)#找到多个色块中的最大色块返回索引值
                    #返回最大色块外框元组(x,y,w,h) 绘制线宽为2的矩形框 不填充矩形
                    TRA_img.draw_rectangle(blobs[maxb_index].rect(), thickness = 2, fill = False)
                    #最大色块的中心位置标记十字
                    TRA_img.draw_cross(blobs[maxb_index].cx(),blobs[maxb_index].cy())
                    #计算质心和=(ROI中最大颜色块的中心点横坐标)cx*(ROI权值)w
                    Centroid_Sum += blobs[maxb_index].cx() * r[4]
                '''else :#如果没找到色块
                    counter0+=1
                    if(counter0>5):
                        counter0=0
                        TRA_img.draw_string(0,0,"Car:\rNo Blobs Find")#LCD显示小车旋转
                        print("NoBlobs Car ")#用于程序调试
                        uart.write(Stop)'''

            #中间公式 确定线心位置=质心和/权值和
            Center_Pos = (Centroid_Sum / Weight_Sum)
            Deflection_Angle = 0#需要将线心Center_Pos转换为偏角 偏角初始化为0
            Deflection_Angle = -math.atan((Center_Pos-80)/60)#计算偏角 限制输出为正负53.13°
            Deflection_Angle = math.degrees(Deflection_Angle)#弧度值转换为角度
            #当偏角大于偏角阈值 且小于最大偏角 和STM32通信 小车左转
            Angle_err=Deflection_Angle
            if abs(Deflection_Angle) > TRA_AngTH:
                if Deflection_Angle>0:

                        uart.write(Right)#和STM32通信 小车左转
                #time.sleep(10)
                        TRA_img.draw_string(0,0,"Car:\rRight")#LCD显示小车运动状态
                        print("Turn Right")#用于程序终端调试
                        print("Turn Angle: %f" % Deflection_Angle)
            #当偏角小于负的偏角阈值 且大于最小偏角
                if Deflection_Angle < 0:
                        uart.write(Left)#和STM32通信 小车右转
                #time.sleep(10)
                        TRA_img.draw_string(0,0,"Car:\rLeft")#LCD显示小车运动状态
                        print("Turn ：Left")#用于程序终端调试
                        print("Turn Angle: %f" % Deflection_Angle)
            #当小车角度绝对值小于阈值
            if abs(Deflection_Angle) <= TRA_AngTH:
                        uart.write(Run)#和STM32通信 小车直行
                        TRA_img.draw_string(0,0,"Car:\rRun")#LCD显示小车运动状态
                        print("Run")#用于程序终端调试
                        print("Turn Angle: %f" % Deflection_Angle)
            '''else:
                Angle_output=Angle_pid.get_pid(Angle_err,1)
                PWMOutL=40+Angle_output
                PWMOutR=40-Angle_output
                Send_PWM(PWMOutR,PWMOutL)'''
            lcd.display(TRA_img)#在LCD上显示img图像
            Recive_Data()       #串口接收数据 若系统状态改变则跳出条件

    elif (OVSys_State == 2) :   #避障模式
                sensor.set_pixformat(sensor.RGB565)#设置摄像头显示图像为彩图
                Hor_servo.angle(0)  #避障模式舵机姿态调整
                Ver_servo.angle(0)  #水平和垂直均是-10
                OBS_img = sensor.snapshot()#镜头畸变校正 去除镜头的鱼眼效果
                OBS_img.mean(1)
                OBS_img.morph(kernel_size, kerne2)
                #在LCD上打印帧率和OpenMV工作模式
                OBS_img.draw_string(0, 20,"OpenMv Mode:\r(2)Obstacle Mode")
                OBS_img.draw_string(0, 0, "FPS:%.2f"%(clock.fps()))
                #寻找色块 (颜色阈值,ROI区域,像素阈值,区域阈值)
                blobs=OBS_img.find_blobs(OBS_TH1,pixels_threshold=100, area_threshold=100,merge=True)
                if blobs:#找到了黄色色块
                    maxb_index =Get_MaxIndex(blobs)#找到最大色块并返回索引值
                    #返回最大色块外框元组(x,y,w,h) 绘制线宽为2的矩形框 不填充矩形
                    OBS_img.draw_rectangle(blobs[maxb_index].rect(), thickness = 2, fill = False)
                    #最大色块的中心位置标记十字
                    OBS_img.draw_cross(blobs[maxb_index].cx(),blobs[maxb_index].cy())
                    maxb= blobs[maxb_index]#定义最大色块为maxb
                    CPix = (maxb[2]+maxb[3])/2#色块的23索引可以获得色块宽度和高度 近似代替实物像素
                    Lm =round((K/CPix),2) #实际距离和像素大小成反比 圆整到小数后两位
                    OBS_img.draw_string(0, 0, "FPS:%.2f"%(clock.fps()))#LCD上显示帧率
                    OBS_img.draw_string(80, 0,"Dis:%.2fcm"%Lm)     #LCD上显示距离
                    print("Dis:%.2fcm"%Lm)
                    if Lm > OBS_DisTH:#如果距离大于阈值
                                OBS_img.draw_string(0, 10,"Distance is Ok")#LCD显示距离正常
                                uart.write(Run)#和STM32通信 距离正常直行
                    if Lm <= OBS_DisTH:#如果距离小于阈值
                                OBS_img.draw_string(0, 10,"Dis is Not Ok ")#LCD显示距离过小
                                uart.write(Stop)
                                time.sleep(1000)
                                uart.write(Right)
                                time.sleep(500)
                                uart.write(Stop)
                                time.sleep(1000)

                else :#如果没找到色块 则距离也是正常 相当于距离大于阈值
                    counter6+=1
                    if counter6>5:
                        counter6=0
                        OBS_img.draw_string(0, 10,"Distance is Ok")#LCD显示距离正常
                        uart.write(Run)
                color_num=0
                blobs1 = OBS_img.find_blobs([Red_TH],pixels_threshold=100)
                if blobs1:
                    color_num=1
                    print("物体颜色:红色")
                    OBS_img.draw_string(0, 40,"Object:\r Red")
                else :
                    blobs2 = OBS_img.find_blobs([Blue_TH],pixels_threshold=100)
                    if blobs2:
                        color_num=2
                        print("物体颜色:蓝色")
                        OBS_img.draw_string(0, 40,"Object:\r Blue")
                lcd.display(OBS_img)#LCD显示图像
                Recive_Data()#串口接收数据 若系统状态改变则跳出条件
    elif (OVSys_State == 4) :
            if Hor_Angle <= 90 and Hor_Angle >= -90:
                Hor_Angle1=Hor_Angle + 15
                Hor_servo.angle(Hor_Angle1)
                Hor_Angle=Hor_Angle1
            if Hor_Angle > 90 or Hor_Angle < -90:
                Hor_servo.angle(0)
                Hor_Angle=0
            OVSys_State = 0
            Recive_Data()
    elif (OVSys_State == 5):
            if Hor_Angle <= 90 and Hor_Angle >= -90:
                Hor_Angle1=Hor_Angle - 15
                Hor_servo.angle(Hor_Angle1)
                Hor_Angle=Hor_Angle1
            if Hor_Angle > 90 or Hor_Angle < -90:
                Hor_servo.angle(0)
                Hor_Angle=0
            OVSys_State = 0
            Recive_Data()
    elif (OVSys_State == 6) :
            if Ver_Angle <= 90 and Ver_Angle >= -90:
                Ver_Angle1=Ver_Angle - 15
                Ver_servo.angle(Ver_Angle1)
                Ver_Angle=Ver_Angle1
            if Ver_Angle > 90 or Ver_Angle < -90:
                Ver_servo.angle(0)
                Ver_Angle=0
            OVSys_State = 0
            Recive_Data()
    elif (OVSys_State == 7) :
            if Ver_Angle <= 90 and Ver_Angle >= -90:
                Ver_Angle1=Ver_Angle + 15
                Ver_servo.angle(Ver_Angle1)
                Ver_Angle=Ver_Angle1
            if Ver_Angle > 90 or Ver_Angle < -90:
                Ver_servo.angle(0)
                Ver_Angle=0
            OVSys_State = 0
            Recive_Data()
    elif (OVSys_State == 8) :
            if TRA_AngTH < 50 and TRA_AngTH > -50:
                TRA_AngTH+=5
            else:
                TRA_AngTH=20
            OVSys_State = 0
            Recive_Data()
    elif (OVSys_State == 9) :
            if TRA_AngTH < 50 and TRA_AngTH > -50:
                TRA_AngTH-=5
            else:
                TRA_AngTH=20
            OVSys_State = 0
            Recive_Data()
    elif (OVSys_State == 10) :
        OBS_DisTH1=OBS_DisTH+0.5
        OBS_DisTH=OBS_DisTH1
        OVSys_State = 0
        Recive_Data()
    elif (OVSys_State == 11) :
        OBS_DisTH1=OBS_DisTH-0.5
        OBS_DisTH=OBS_DisTH1
        OVSys_State = 0
        Recive_Data()
    elif (OVSys_State == 12) :
        PValue1=PValue-0.01
        PValue=PValue1
        OVSys_State = 0
        Recive_Data()
    elif (OVSys_State == 13) :
        PValue1=PValue+0.01
        PValue=PValue1
        OVSys_State = 0
        Recive_Data()
    elif (OVSys_State == 14) :
        IValue1=IValue+0.001
        IValue=IValue1
        OVSys_State = 0
        Recive_Data()
    elif (OVSys_State == 15) :
        IValue1=IValue-0.001
        IValue=IValue1
        OVSys_State = 0
        Recive_Data()
    else :#默认模式
            sensor.set_pixformat(sensor.RGB565)#设置摄像头显示图像为彩图
            img = sensor.snapshot()#截取一帧图片
            img.draw_string(0, 0, "FPS:%.2f"%(clock.fps()))#LCD上显示帧率
            img.draw_string(0, 20, "Wait Mode  Change\rOpenMV now no mode")#LCD显示系统模式
            img.draw_string(0, 40, "Hor_Ang:%d"%(Hor_Angle))#LCD上显示帧率
            img.draw_string(0, 50, "Ver_Ang:%d"%(Ver_Angle))#LCD上显示帧率
            img.draw_string(0, 60, "Ang_TH:%d"%(TRA_AngTH))#LCD上显示帧率
            img.draw_string(0, 70, "Dis_TH:%.1fcm"%(OBS_DisTH))#LCD上显示帧率
            img.draw_string(0, 80, "PID PValue:%.2f"%(PValue))#LCD上显示帧率
            img.draw_string(0, 90, "PID IValue:%.3f"%(IValue))#LCD上显示帧率
            #img.morph(kernel_size, kerne1)
            lcd.display(img)#LCD显示图像
            Recive_Data()#串口接收数据 若系统状态改变则跳出条件
    Recive_Data()#串口接收数据 若系统状态改变则进入上述三种条件
