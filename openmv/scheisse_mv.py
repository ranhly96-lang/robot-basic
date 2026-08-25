class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp  # 比例增益
        self.Ki = Ki  # 积分增益
        self.Kd = Kd  # 微分增益
        self.prev_error = 0  # 上次误差
        self.integral = 0  # 积分项

    def compute(self, setpoint, measured_value):
        # 计算误差
        error = setpoint - measured_value

        # 积分项
        self.integral += error

        # 微分项
        derivative = error - self.prev_error

        # PID 输出
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # 更新前一次误差
        self.prev_error = error

        return output
import sensor, image, time  # 导入必要库
import math
from pyb import LED
from pyb import Pin, delay

from machine import UART
clock = time.clock()
uart = UART(3, 115200)
# 初始化 GPIO
pin0 = Pin('P0', Pin.IN)
pin1=  Pin('P1', Pin.OUT)
pin2 = Pin('P2', Pin.OUT)
pin3 = Pin('P3', Pin.OUT)
pin4 = Pin('P7', Pin.OUT)
pin5 = Pin('P8', Pin.OUT)
# 初始化
sensor.reset()  # 初始化感光器
sensor.set_framesize(sensor.QVGA)  # 设置感光器分辨率
# sensor.set_pixformat(sensor.GRAYSCALE)  # 设置黑白模式
sensor.set_pixformat(sensor.RGB565)  # 设置为彩色模式

LED(1).on()
LED(2).on()
LED(3).on()
# 初始化PID控制器
pid_x = PID(1.8, 0, 1.8)
pid_turn = PID(1.4,0, 1.4)
pid_pick_1 = PID(1.6, 0.07, 1)

# 初始化变量
threshold = [[0, 128] for _ in range(6)]  # 阈值列表
midpoint = 160  # 屏幕中心值
line_point = [0] * 6  # 记录每段黑线的偏移量
line_area = [[0, 0, 0, 0]] * 6  # 记录每段黑线的区域信息
line_none = 0  # 无黑线计数

# 定义传输函数
def int_to_bool_list_with_sign_at_position(number, sign_position=6):
    """
    将整数的每一位按位提取为布尔值，并将符号位设置在指定的位置。
    """
    if not isinstance(number, int):
        raise ValueError("输入值必须是整数")
    # 判断是否为正数
    is_positive = number >= 0
    # 取绝对值
    abs_number = abs(number)
    # 获取二进制表示（去掉 '0b' 前缀）
    binary_representation = bin(abs_number)[2:]
    # 按位转换为布尔值列表
    bool_list = [bit == '1' for bit in reversed(binary_representation)]
    # 如果符号位位置小于列表长度，插入符号位
    if sign_position < len(bool_list):
        bool_list.insert(sign_position, is_positive)
    else:
        # 如果符号位位置超过当前长度，填充零并插入符号位
        bool_list.extend([False] * (sign_position - len(bool_list)))
        bool_list[5] = is_positive
    return bool_list

def send_a_5bit(data):
    """
    传输 6 位数据
    """
    bool_list = int_to_bool_list_with_sign_at_position(data, sign_position=5)
    if bool_list[0]:
        pin1.high()  # 设置为高电平
    else:
        pin1.low()
    if bool_list[1]:
        pin2.high()
    else:
        pin2.low()
    if bool_list[2]:
        pin3.high()
    else:
        pin3.low()
    if bool_list[3]:
        pin4.high()
    else:
        pin4.low()
    if bool_list[4]:
        pin5.high()
    else:
        pin5.low()
    return 0

def send_to_chassis(vx,vy,wz):
    a_wheel=int((vx+vy-wz)*1)
    b_wheel=int((vx-vy-wz)*1)
    c_wheel=int((vx+vy+wz)*1)
    d_wheel=int((vx-vy+wz)*1)
    cmd_return_tmp = f"#{8:03d}P{1500 + a_wheel:04d}T{0:04d}!"
    uart.write(cmd_return_tmp)
    cmd_return_tmp = f"#{6:03d}P{1500 + b_wheel:04d}T{0:04d}!"
    uart.write(cmd_return_tmp)
    cmd_return_tmp = f"#{7:03d}P{1500 - c_wheel:04d}T{0:04d}!"
    uart.write(cmd_return_tmp)
    cmd_return_tmp = f"#{9:03d}P{1500 - d_wheel:04d}T{0:04d}!"
    uart.write(cmd_return_tmp)
pin1.low()
pin2.low()
pin3.low()
pin4.low()
pin5.low()
take_a_look_flag=0
change_arm_mode=0#第1,2,3号调节模式
total_big_line_cmd=1

sharp_turn_num=0
sharp_turn_count=0
time_after_st=0
second_st_flag=0
second_st_count=0

pick_color=0#0没有抓到 1红 2绿 3蓝

change_arm_mode=0

green_count=0

uart_data=0

no_double_st_count=170

over_time_count=170

finish_count=0

write_cha_flag=0

nothing_on_box_count=100

last_pick_color=0

# 主循环
while True:
    img = sensor.snapshot()  # 获取当前帧图像
    img_copy=img.copy()#备份

    # 将 img 转换为灰度图像并保持 img 不变
    gray_channel = img.copy().to_grayscale()  # 创建一个灰度图像副本
    gray_channel.binary([(90, 255)])  # 二值化：
    if total_big_line_cmd==1:
        ##############################下面是整体灰度色块识别#################################
        big_blobs=gray_channel.find_blobs([(0, 0)], area_threshold=3000)#找超大色块
        if big_blobs:
            largest_blob = max(big_blobs, key=lambda b: b.area())  # 按面积排序
            # gray_channel.draw_rectangle(largest_blob.rect(), color=(255, 0, 0))  # 红色矩形
    ###############################取一行分块#####################################
    dots_num=11
    dotsw=(gray_channel.width()//11)
    dotsh=70
    gray_m_row=gray_channel.copy()
    gray_m_row.crop(roi=(0,57,gray_channel.width(),dotsh))
    black_dots=[]
    for i in range(dots_num):
        temp_dot=gray_m_row.copy()
        temp_dot.crop(roi=(i*dotsw,0,dotsw,dotsh))
        temp_blobs=temp_dot.find_blobs([(0, 0)], area_threshold=100)
        if temp_blobs:
            black_dots.append(1)
            gray_m_row.draw_cross(i*dotsw+temp_blobs[0].cx(),temp_blobs[0].cy())
        else:
            black_dots.append(0)
    left_byte=0
    right_byte=0
    left_conti_len=0
    right_conti_len=0
    for i in range(dots_num//2-1):
        left_byte=left_byte+((dots_num//2)-i)*black_dots[i]
        if black_dots[dots_num//2-i]:
            left_conti_len=left_conti_len+1
    for i in range(dots_num//2-1):
        right_byte=right_byte+((dots_num//2)-i)*black_dots[dots_num-1-i]
        if black_dots[dots_num//2+i]:
            right_conti_len=right_conti_len+1
    sharp_left_turn_flag=0
    sharp_right_turn_flag=0
    if left_byte>9 and left_byte>right_byte and left_conti_len>2 :
        sharp_left_turn_flag=1
    if right_byte>9 and right_byte>left_byte and right_conti_len>2:
        sharp_right_turn_flag=1
    ################################下面是循迹识别+pid计算#####################################

    block_count = 10
    height = gray_channel.height()
    block_height = height // block_count  # 每个块的高度
    blocks_gray = []  # 用于保存分块后的数据
    cx_total=[]
    cy_total=[]
    cxy_num=0
    # 按行分块
    for i in range(block_count):
        y_start = i * block_height  # 当前块的起始y坐标
        block = gray_channel.copy()
        block.crop(roi=(0, y_start, gray_channel.width(), block_height))
        # 将块保存到列表
        blocks_gray.append(block)
        if i>=(0):
            blobs = blocks_gray[i].find_blobs([(0, 0)], area_threshold=200)  # 找到黑色区域 一般是1个
            if len(blobs)>=1:
                for j in range(len(blobs)):
                    if j==1:
                        break
                    temp_cx=blobs[j].cx()
                    temp_cy=blobs[j].cy()+i*block_height
                    # if blobs[j].w()>70:
                    #     if (temp_cx-gray_channel.width()//2)>10:
                    #         temp_cx=temp_cx+700
                    #         vx_0_flag=1
                    #         vx_0_count=0
                    #     elif (temp_cx-gray_channel.width()//2)<-10:
                    #         temp_cx=temp_cx-700
                    #         vx_0_flag=1
                    #         vx_0_count=0
                    cx_total.append(temp_cx)
                    cy_total.append(temp_cy)
                    cxy_num=cxy_num+1
            #补丁1
            if len(blobs)>=1 and i<=1:
                sharp_left_turn_flag=0
                sharp_right_turn_flag=0

    cx_average=0
    cy_average=0
    cx_average_2=0
    cy_average_2=0
    cx_sum=0#用于计算两个平均值
    cy_sum=0;
    if cxy_num!=0:
        for i in range(cxy_num//4):
            cx_sum=cx_sum+cx_total[i]
            cy_sum=cy_sum+cy_total[i]
        if (cxy_num//4)==0:
            cx_average=cx_sum
            cy_average=cy_sum
        else:
            cx_average=int(cx_sum/(cxy_num//4))
            cy_average=int(cy_sum/(cxy_num//4))
        gray_channel.draw_cross(cx_average,cy_average)  # 绘制交叉线，表示中心
        for i in range(cxy_num):
            gray_channel.draw_cross(cx_total[i],cy_total[i])  # 绘制交叉线，表示中心
    img.replace(gray_channel)
    half_width=(gray_channel.width()//2)
    turn_output=pid_turn.compute(half_width,cx_average)
    x_output=0
    if turn_output>300:
        turn_output=300+(turn_output-300)*0.5
    elif turn_output<-300:
        turn_output=-300+(turn_output+300)*0.5
    if cxy_num!=0:
        x_output=pid_x.compute(half_width,cx_total[cxy_num-1])
        if x_output>400:
            x_output=400+(x_output-400)*0.5
        elif x_output<-400:
            x_output=-400+(x_output+400)*0.5

    ###############################下面是彩色识别######################################
    blobs_red = img_copy.find_blobs([(0, 100, 15, 127, 15, 127)], pixels_threshold=200, area_threshold=200, merge=True)
    blobs_blue = img_copy.find_blobs([(0, 120, -127, 127, -127, -15)], pixels_threshold=200, area_threshold=200, merge=True)
    blobs_green = img_copy.find_blobs([(0, 100, -127, -15, -127, 127)], pixels_threshold=200, area_threshold=200, merge=True)
    if blobs_red:
        blob_red=max(blobs_red, key=lambda b: b.area())
        # 绘制矩形框和中心点
        img_copy.draw_rectangle(blob_red.rect())  # 绘制识别区域的矩形
        img_copy.draw_cross(blob_red.cx(), blob_red.cy())  # 绘制识别区域的中心点
    if blobs_blue:
        blob_blue=max(blobs_blue, key=lambda b: b.area())
        # 绘制矩形框和中心点
        img_copy.draw_rectangle(blob_blue.rect())  # 绘制识别区域的矩形
        img_copy.draw_cross(blob_blue.cx(), blob_blue.cy())  # 绘制识别区域的中心点
    if blobs_green:
        blob_green=max(blobs_green, key=lambda b: b.area())
        # 绘制矩形框和中心点
        img_copy.draw_rectangle(blob_green.rect())  # 绘制识别区域的矩形
        img_copy.draw_cross(blob_green.cx(), blob_green.cy())  # 绘制识别区域的中心点
    ##################################double st计数################################
    if time_after_st>0:
        time_after_st=time_after_st-1
    else:
        time_after_st=0
    if no_double_st_count>0:
        no_double_st_count=no_double_st_count-1
    else:
        no_double_st_count=0
    if no_double_st_count==0:
        #识别双转弯
        if sharp_left_turn_flag or sharp_right_turn_flag:
            if time_after_st>0:
                second_st_count=second_st_count+1
                if sharp_left_turn_flag:
                    second_st_flag=1
                if sharp_right_turn_flag:
                    second_st_flag=2
            else:
                second_st_flag=0
            time_after_st=70
        # pin1.high()#告诉arduino调节机械臂
        # pin2.low()
        if change_arm_mode==0:
            if time_after_st>0:
                time_after_st=time_after_st-1
                # if pin0.value()==1:
                if second_st_flag:
                    if pick_color==0:
                        change_arm_mode=1
                        time_after_st=0
                        pin1.high()#告诉arduino调节机械臂
                        pin2.low()
                        wz_temp=0
                        if second_st_flag==1:
                            wz_temp=350
                        if second_st_flag==2:
                            wz_temp=-350
                        send_to_chassis(-300,0,-wz_temp)
                        time.sleep(-1.5)
                        send_to_chassis(-270,-100,0)
                        time.sleep(1.5)
                        #补丁2
                        if second_st_flag==1:
                            send_to_chassis(0,0,400)
                            time.sleep(1.7-0.1)
                            send_to_chassis(0,-270,0)
                            time.sleep(0.6)
                    if pick_color==3:
                        change_arm_mode=2
                        time_after_st=0
                        pin1.low()#告诉arduino调节机械臂
                        pin2.high()
                        wz_temp=0
                        if second_st_flag==1:
                            wz_temp=350
                        if second_st_flag==2:
                            wz_temp=-350
                        send_to_chassis(-300,0,-wz_temp)
                        time.sleep(-1.5)
                        send_to_chassis(-270,-100,0)
                        time.sleep(1.5)
                        send_to_chassis(0,-200,0)
                        time.sleep(1)
                        send_to_chassis(0,0,0)
                        time.sleep(1.5)
                    if pick_color==2:
                        change_arm_mode=2
                        time_after_st=0
                        pin1.low()#告诉arduino调节机械臂
                        pin2.high()
                        send_to_chassis(-300,0,-wz_temp)
                        time.sleep(-1.5)
                        send_to_chassis(-270,-100,0)
                        time.sleep(1.5)
                        send_to_chassis(0,-200,0)
                        time.sleep(0.3)

                    if pick_color==1:
                        change_arm_mode=3
                        time_after_st=0
                        pin1.high()#告诉arduino调节机械臂
                        pin2.high()
                        send_to_chassis(-300,0,-wz_temp)
                        time.sleep(-1.5)
                        send_to_chassis(-270,-100,0)
                        time.sleep(1.5)
                    second_st_flag=0
            else:
                time_after_st=0
        #由完成工作与否决定change_arm_mode复位
    # else:
    #     change_arm_mode=0

    if change_arm_mode==1:#第一调节模式
        total_big_line_cmd=0
        img.replace(img_copy)
        pin1.high()#告诉arduino调节机械臂
        pin2.low()
         #如果是红色
        if blobs_red:
            nothing_on_box_count=100
            pick_color=1
            if abs(blob_red.cy()-(img_copy.height()//2))<=80:
                if blob_red.area()-10000>1000:
                    print('back')
                    pin3.high()
                    pin4.high()
                    pin5.low()
                    uart_data=(1)*1+(2**1)*0+(2**2)*1+(2**3)*1+(2**4)*0
                    uart.write(bytes([0xAA]))
                    uart.write(bytes([uart_data]))
                    uart.write(bytes([0xBB]))
                if blob_red.area()-10000<-1000:
                    print('forward')
                    pin3.low()
                    pin4.low()
                    pin5.high()
                    uart_data=(1)*1+(2**1)*0+(2**2)*0+(2**3)*0+(2**4)*1
                    uart.write(bytes([0xAA]))
                    uart.write(bytes([uart_data]))
                    uart.write(bytes([0xBB]))
            else:
                    #通道不够,优先调高度
                if blob_red.cy()-(img_copy.height()//2)<-20:
                    print('up')
                    pin3.high()
                    pin4.low()
                    pin5.high()
                    uart_data=(1)*1+(2**1)*0+(2**2)*1+(2**3)*0+(2**4)*1
                    uart.write(bytes([0xAA]))
                    uart.write(bytes([uart_data]))
                    uart.write(bytes([0xBB]))
                if blob_red.cy()-(img_copy.height()//2)>20:
                    print('down')
                    pin3.high()
                    pin4.low()
                    pin5.low()
                    uart_data=(1)*1+(2**1)*0+(2**2)*1+(2**3)*0+(2**4)*0
                    uart.write(bytes([0xAA]))
                    uart.write(bytes([uart_data]))
                    uart.write(bytes([0xBB]))
            pick_vy_=0
            if blob_red.area()-12000<-3700:
                pick_vy_=-200
                pick_vy_=0#取消
            pick_vx=pid_pick_1.compute((img_copy.width()//2),blob_red.cx())
            send_to_chassis(pick_vx,pick_vy_,0)
            if  (abs(blob_red.cx()-(img_copy.width()//2))<10 and abs(blob_red.cy()-(img_copy.height()//2))<=30 and abs(blob_red.area()-10000)<1000):
                print('ok')
                #6表示 a little ok
                pin3.low()
                pin4.high()
                pin5.high()
                uart_data=(1)*1+(2**1)*0+(2**2)*0+(2**3)*1+(2**4)*1
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                send_to_chassis(0,0,0)
                time.sleep(3.5)
                #全高(7)表示ok
                pin3.high()
                pin4.high()
                pin5.high()
                uart_data=(1)*1+(2**1)*0+(2**2)*1+(2**3)*1+(2**4)*1
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                send_to_chassis(0,0,0)
                time.sleep(1.5)
                #(二货)2表示除了yaw,其他轴还原为循迹(怕磕到设备),这段时间也可以调整车的朝向
                pin3.low()
                pin4.high()
                pin5.low()
                uart_data=(1)*1+(2**1)*0+(2**2)*0+(2**3)*1+(2**4)*0
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                send_to_chassis(0,240,0)
                time.sleep(1)
                pin1.low()
                pin2.low()
                pin3.low()
                pin4.low()
                pin5.low()
                uart_data=(1)*0+(2**1)*0+(2**2)*0+(2**3)*0+(2**4)*0
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                send_to_chassis(0,0,400)
                time.sleep(1.7)
                change_arm_mode=0
                time_after_st=0
                second_st_flag=0
                no_double_st_count=130

                over_time_count=170

                finish_count=finish_count+1
                if finish_count==3:
                    write_cha_flag=1
            else:
                over_time_count=over_time_count-1
        elif blobs_blue:
            pick_color=3
            nothing_on_box_count=100
            if abs(blob_blue.cy()-(img_copy.height()//2))<=80:
                if blob_blue.area()-10000>1000:
                    print('back')
                    pin3.high()
                    pin4.high()
                    pin5.low()
                    uart_data=(1)*1+(2**1)*0+(2**2)*1+(2**3)*1+(2**4)*0
                    uart.write(bytes([0xAA]))
                    uart.write(bytes([uart_data]))
                    uart.write(bytes([0xBB]))
                if blob_blue.area()-10000<-1000:
                    print('forward')
                    pin3.low()
                    pin4.low()
                    pin5.high()
                    uart_data=(1)*1+(2**1)*0+(2**2)*0+(2**3)*0+(2**4)*1
                    uart.write(bytes([0xAA]))
                    uart.write(bytes([uart_data]))
                    uart.write(bytes([0xBB]))
            else:
                    #通道不够,优先调高度
                if blob_blue.cy()-(img_copy.height()//2)<-20:
                    print('up')
                    pin3.high()
                    pin4.low()
                    pin5.high()
                    uart_data=(1)*1+(2**1)*0+(2**2)*1+(2**3)*0+(2**4)*1
                    uart.write(bytes([0xAA]))
                    uart.write(bytes([uart_data]))
                    uart.write(bytes([0xBB]))
                if blob_blue.cy()-(img_copy.height()//2)>20:
                    print('down')
                    pin3.high()
                    pin4.low()
                    pin5.low()
                    uart_data=(1)*1+(2**1)*0+(2**2)*1+(2**3)*0+(2**4)*0
                    uart.write(bytes([0xAA]))
                    uart.write(bytes([uart_data]))
                    uart.write(bytes([0xBB]))
            pick_vy_=0
            if blob_blue.area()-12000<-3700:
                pick_vy_=-200
                pick_vy_=0#取消
            pick_vx=pid_pick_1.compute((img_copy.width()//2),blob_blue.cx())
            send_to_chassis(pick_vx,pick_vy_,0)
            if  abs(blob_blue.cx()-(img_copy.width()//2))<10 and abs(blob_blue.cy()-(img_copy.height()//2))<=30 and abs(blob_blue.area()-10000)<1000:
                print('ok')
                #6表示 a little ok
                pin3.low()
                pin4.high()
                pin5.high()
                uart_data=(1)*1+(2**1)*0+(2**2)*0+(2**3)*1+(2**4)*1
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                send_to_chassis(0,0,0)
                time.sleep(3.5)
                #全高(7)表示ok
                pin3.high()
                pin4.high()
                pin5.high()
                uart_data=(1)*1+(2**1)*0+(2**2)*1+(2**3)*1+(2**4)*1
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                send_to_chassis(0,0,0)
                time.sleep(1.5)
                #(二货)2表示除了yaw,其他轴还原为循迹(怕磕到设备),这段时间也可以调整车的朝向
                pin3.low()
                pin4.high()
                pin5.low()
                uart_data=(1)*1+(2**1)*0+(2**2)*0+(2**3)*1+(2**4)*0
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                send_to_chassis(0,240,0)
                time.sleep(1)
                pin1.low()
                pin2.low()
                pin3.low()
                pin4.low()
                pin5.low()
                uart_data=(1)*0+(2**1)*0+(2**2)*0+(2**3)*0+(2**4)*0
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                send_to_chassis(0,0,0)
                time.sleep(1)
                change_arm_mode=0
                time_after_st=0
                second_st_flag=0
                no_double_st_count=130

                over_time_count=130

                finish_count=finish_count+1
                if finish_count==3:
                    write_cha_flag=1
            else:
                over_time_count=over_time_count-1;
        elif blobs_green:
            pick_color=2
            nothing_on_box_count=100
            if abs(blob_green.cy()-(img_copy.height()//2))<=80:
                if blob_green.area()-10000>1000:
                    print('back')
                    pin3.high()
                    pin4.high()
                    pin5.low()
                    uart_data=(1)*1+(2**1)*0+(2**2)*1+(2**3)*1+(2**4)*0
                    uart.write(bytes([0xAA]))
                    uart.write(bytes([uart_data]))
                    uart.write(bytes([0xBB]))
                if blob_green.area()-10000<-1000:
                    print('forward')
                    pin3.low()
                    pin4.low()
                    pin5.high()
                    uart_data=(1)*1+(2**1)*0+(2**2)*0+(2**3)*0+(2**4)*1
                    uart.write(bytes([0xAA]))
                    uart.write(bytes([uart_data]))
                    uart.write(bytes([0xBB]))
            else:
                    #通道不够,优先调高度
                if blob_green.cy()-(img_copy.height()//2)<-20:
                    print('up')
                    pin3.high()
                    pin4.low()
                    pin5.high()
                    uart_data=(1)*1+(2**1)*0+(2**2)*1+(2**3)*0+(2**4)*1
                    uart.write(bytes([0xAA]))
                    uart.write(bytes([uart_data]))
                    uart.write(bytes([0xBB]))
                if blob_green.cy()-(img_copy.height()//2)>20:
                    print('down')
                    pin3.high()
                    pin4.low()
                    pin5.low()
                    uart_data=(1)*1+(2**1)*0+(2**2)*1+(2**3)*0+(2**4)*0
                    uart.write(bytes([0xAA]))
                    uart.write(bytes([uart_data]))
                    uart.write(bytes([0xBB]))
            pick_vy_=0
            if blob_green.area()-12000<-3700:
                pick_vy_=-200
                pick_vy_=0#取消
            pick_vx=pid_pick_1.compute((img_copy.width()//2),blob_green.cx())
            send_to_chassis(pick_vx,pick_vy_,0)
            if  abs(blob_green.cx()-(img_copy.width()//2))<10 and abs(blob_green.cy()-(img_copy.height()//2))<=30 and abs(blob_green.area()-10000)<1000:
                print('ok')
                #6表示 a little ok
                pin3.low()
                pin4.high()
                pin5.high()
                uart_data=(1)*1+(2**1)*0+(2**2)*0+(2**3)*1+(2**4)*1
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                send_to_chassis(0,0,0)
                time.sleep(3.5)
                #全高(7)表示ok
                pin3.high()
                pin4.high()
                pin5.high()
                uart_data=(1)*1+(2**1)*0+(2**2)*1+(2**3)*1+(2**4)*1
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                send_to_chassis(0,0,0)
                time.sleep(1.5)
                #(二货)2表示除了yaw,其他轴还原为循迹(怕磕到设备),这段时间也可以调整车的朝向
                pin3.low()
                pin4.high()
                pin5.low()
                uart_data=(1)*1+(2**1)*0+(2**2)*0+(2**3)*1+(2**4)*0
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                send_to_chassis(0,240,0)
                time.sleep(1)
                pin1.low()
                pin2.low()
                pin3.low()
                pin4.low()
                pin5.low()
                uart_data=(1)*0+(2**1)*0+(2**2)*0+(2**3)*0+(2**4)*0
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                send_to_chassis(0,0,0)
                time.sleep(1)
                change_arm_mode=0
                time_after_st=0
                second_st_flag=0
                no_double_st_count=400

                over_time_count=170

                finish_count=finish_count+1
                if finish_count==3:
                    write_cha_flag=1
            else:
                over_time_count=over_time_count-1
        else:
            send_to_chassis(0,0,0)
            pin3.low()
            pin4.low()
            pin5.low()
            uart_data=(1)*1+(2**1)*0+(2**2)*0+(2**3)*0+(2**4)*0
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            nothing_on_box_count=nothing_on_box_count-1
            if nothing_on_box_count==0:
                if last_pick_color==1:
                    uart_data=0xcc
                elif last_pick_color==2:
                    uart_data=0xcd
                elif last_pick_color==3:
                    uart_data=0xce
                else:
                    uart_data=0xcc
                send_to_chassis(0,0,400)
                time.sleep(0.9)
                send_to_chassis(400,0,0)
                time.sleep(1.9)
                send_to_chassis(0,0,0)
                time.sleep(0.9)
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                uart.write(bytes([0xAA]))
                uart.write(bytes([uart_data]))
                uart.write(bytes([0xBB]))
                time.sleep(100.9)
    elif change_arm_mode==2:
        total_big_line_cmd=0
        img.replace(gray_channel)
        pin1.low()#告诉arduino调节机械臂
        pin2.high()
         #如果是黑块
        if big_blobs:
            print('forward')
            pin3.low()
            pin4.low()
            pin5.high()
            uart_data=(1)*0+(2**1)*1+(2**2)*0+(2**3)*0+(2**4)*1
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            send_to_chassis(0,0,0)
            time.sleep(1.2)
            print('ok')
            #6表示 a little ok
            pin3.low()
            pin4.high()
            pin5.high()
            uart_data=(1)*0+(2**1)*1+(2**2)*0+(2**3)*1+(2**4)*1
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            send_to_chassis(0,0,0)
            time.sleep(2.7)
            #全高(7)表示ok
            pin3.high()
            pin4.high()
            pin5.high()
            uart_data=(1)*0+(2**1)*1+(2**2)*1+(2**3)*1+(2**4)*1
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            send_to_chassis(0,0,0)
            time.sleep(1)
            #(二货)2表示除了yaw,其他轴还原为循迹(怕磕到设备),这段时间也可以调整车的朝向
            pin3.low()
            pin4.high()
            pin5.low()
            uart_data=(1)*0+(2**1)*1+(2**2)*0+(2**3)*1+(2**4)*0
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            send_to_chassis(0,0,400)
            time.sleep(1.7)
            uart_data=(1)*0+(2**1)*0+(2**2)*0+(2**3)*1+(2**4)*0
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart_data=(1)*0+(2**1)*0+(2**2)*0+(2**3)*0+(2**4)*0
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            send_to_chassis(0,0,0)
            time.sleep(1.5)
            if pick_color==2:
                no_double_st_count=450
            else:
                no_double_st_count=170
            last_pick_color=pick_color
            pick_color=0
            change_arm_mode=0
        else:
            send_to_chassis(0,0,0)
            pin3.low()
            pin4.low()
            pin5.low()
            uart_data=(1)*0+(2**1)*1+(2**2)*0+(2**3)*0+(2**4)*0
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
    elif change_arm_mode==3:
        total_big_line_cmd=0
        img.replace(gray_channel)
        pin1.high()#告诉arduino调节机械臂
        pin2.high()
         #如果是黑块
        if big_blobs:
            print('forward')
            pin3.low()
            pin4.low()
            pin5.high()
            uart_data=(1)*1+(2**1)*1+(2**2)*0+(2**3)*0+(2**4)*1
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))

            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            send_to_chassis(0,300,0)
            time.sleep(1)
            ######################
            pin3.low()
            pin4.high()
            pin5.high()
            uart_data=(1)*1+(2**1)*1+(2**2)*0+(2**3)*1+(2**4)*1
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            send_to_chassis(0,0,0)
            time.sleep(1.2)
            #全高(7)表示ok
            pin3.high()
            pin4.high()
            pin5.high()
            uart_data=(1)*1+(2**1)*1+(2**2)*1+(2**3)*1+(2**4)*1
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            send_to_chassis(0,0,0)
            time.sleep(1.5)
            #(二货)2表示除了yaw,其他轴还原为循迹(怕磕到设备),这段时间也可以调整车的朝向
            pin3.low()
            pin4.high()
            pin5.low()
            uart_data=(1)*1+(2**1)*1+(2**2)*0+(2**3)*1+(2**4)*0
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            send_to_chassis(0,-270,0)
            time.sleep(1)
            send_to_chassis(0,0,400)
            time.sleep(1.7-0.3)
            uart_data=(1)*0+(2**1)*0+(2**2)*0+(2**3)*0+(2**4)*0
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
            send_to_chassis(0,0,0)
            time.sleep(1.5)
            last_pick_color=pick_color
            pick_color=0
            change_arm_mode=0
            no_double_st_count=170
        else:
            send_to_chassis(0,0,0)
            pin3.low()
            pin4.low()
            pin5.low()
            uart_data=(1)*1+(2**1)*1+(2**2)*0+(2**3)*0+(2**4)*0
            uart.write(bytes([0xAA]))
            uart.write(bytes([uart_data]))
            uart.write(bytes([0xBB]))
    else:#循迹模式
        pin1.low()
        pin2.low()
        pin3.low()
        pin4.low()
        pin5.low()
        uart_data=(1)*0+(2**1)*0+(2**2)*0+(2**3)*0+(2**4)*0
        total_big_line_cmd=1
        vx=477-abs(turn_output)*1.5
        if sharp_left_turn_flag:
            vx=300
            send_to_chassis(vx,0,700)
            time.sleep(0.31)
        elif sharp_right_turn_flag:
            vx=300
            send_to_chassis(vx,0,-700)
            time.sleep(0.31)
        send_to_chassis(vx,x_output,turn_output)
        # send_to_chassis(0,0,0)
        img.replace(img_copy)
        uart.write(bytes([0xAA]))
        uart.write(bytes([uart_data]))
        uart.write(bytes([0xBB]))





















