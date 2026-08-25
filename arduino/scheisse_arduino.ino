#include <Servo.h>  // 引入舵机库
#include "math.h"

// #define PIN_xj0  A4
// #define PIN_xj1  A5
// 拔掉红外了
//A1 D2 A3 A0 A5 A4
#define pin0 A0
#define pin1 A3
#define pin2 A4
#define pin3 A5
#define pin4 A7
#define pin5 2
#define TrigPin A1
#define EchoPin A2
long Echo_time = 0; 
double distance = 0;
static int count;
static int therehold = 1;//超声波变量
int data_mv=0;//解码后的数据

float xyz_change[3]={0,0,0};
float z_down=0;

uint8_t uart_data=0;
uint8_t buffer[100]={0};
uint8_t bits_from_mv[6]={0};
Servo myServo[6];      // 创建一个舵机对象
char cmd_return_tmp[64];
#define sb_ratio (180.0/270)
#define pi 3.1415926f
#define arm_speed 1000
#define motion_num 12
static uint32_t task_count=0;
// static int theta[6]={0,-45,45,0,200,86};
static int theta[6]={0,-45,45,0,227,86};
static int compensation[6]={-9,20,-3,-1,0,0};
static int vx=0;
static int vy=0;
static int wz=0;
int car_speed=400;
static float target_xyz[3]={0};
static int ik_thetas[motion_num][4]={ {-40,-80,50,14},//1ok
                              {-30,-85,55,14},//2ok
                              {-20,-94,63,17},//3ok
                              {-10,-99,70,18},//4ok
                              {-5,-105,72,20},//5ok
                              {-2,-106,74,22},//6ok
                              {2,-106,74,22},//7
                              {5,-105,72,20},//8
                              {10,-99,70,18},//9
                              {20,-94,63,17},//10
                              {30,-85,55,14},
                              {40,-80,50,14}};//12ok
static float motion_period[motion_num]={0.5,0.4,0.4,0.3,0.2,0.1,0.1,0.2,0.3,0.4,0.4,0.5};//每一个动作的持续时间
static float passed_time[motion_num]={0};//每一个动作已经经过的时间
static int cur_motion=0;//当前动作
void scara_inverse_kinematics(float x, float y, float L1, float L2, uint8_t handcoor, float angles[2]);
void xyz2xztheta(float x,float y ,float z,float* x_,float* z_,float* theta_);
void total_ik(float x,float y, float z,float* j1,float* j2,float* j3,int len1,int len2);
void getLen2_j3bais(float j4,int* len2,float* j3bais);

void setup(){
  Serial.begin(115200);
  myServo[0].attach(7);  // 将舵机连接到数字引脚9
  myServo[1].attach(3);  // 将舵机连接到数字引脚9
  myServo[2].attach(8);  // 将舵机连接到数字引脚9
  myServo[3].attach(6);  // 将舵机连接到数字引脚9
  myServo[4].attach(5);  // 将舵机连接到数字引脚9
  myServo[5].attach(9);  // 将舵机连接到数字引脚9
  //规定pin01为arduino输出
  pinMode(pin0,OUTPUT);
  digitalWrite(pin0,LOW);
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  // pinMode(pin1,INPUT);
  // pinMode(pin2,INPUT);
  // pinMode(pin3,INPUT);
  // pinMode(pin4,INPUT);//模拟引脚不能
  pinMode(pin5,INPUT);
  /*循迹机械臂初始值*/
        theta[0]=0;
        theta[1]=-80;
        theta[2]=45;
        theta[3]=105;
        theta[4]=227;
        theta[5]=75;
  int user_cmd[6]={270/2+theta[0],theta[1]+180,theta[2],theta[3]+85,theta[4],theta[5]};
  myServo[0].write((user_cmd[0]-compensation[0])*sb_ratio);
  myServo[1].write((user_cmd[1]-compensation[1])*sb_ratio);
  myServo[2].write((user_cmd[2]-compensation[2])*sb_ratio);
  myServo[3].write((user_cmd[3]-compensation[3])*sb_ratio);
  myServo[4].write((user_cmd[4]-compensation[4])*sb_ratio);
  myServo[5].write((user_cmd[5]-compensation[5])*sb_ratio);
}

void loop(){
  // ////////////////////////超声波识别是否停止/////////////////////////////////////////
  // digitalWrite(TrigPin, LOW);
  // delayMicroseconds(2);
  // digitalWrite(TrigPin, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(TrigPin, LOW);
  // Echo_time = pulseIn(EchoPin, HIGH);
  // if (Echo_time > 1 && Echo_time < 85000) {
  //   distance = Echo_time*0.017; // cm为单位
  // } else {
  //   distance = 1;
  // }
  // // printPin0State();
  //  if (distance < 37.0 && distance > 20.0) 
  //  {
  //   count++;
  //   if(count >= therehold)
  //     {digitalWrite(pin0,HIGH);}//距离ready
  //   else
  //   {digitalWrite(pin0,LOW);}//距离not ready
  //   }
  //   else
  //   {count=0;digitalWrite(pin0,LOW);}//距离not ready
    /**********************上面为检测,下面为执行openmv命令***************************/
    /*循迹机械臂初始值*/
        theta[0]=0;
        theta[1]=-80;
        theta[2]=45;
        theta[3]=105;
        theta[4]=227;
        theta[5]=75;
    //接收信号
          // int analog1=analogRead(pin1);
          // uint8_t bit_pin1=0;
          // if(analog1>437)
          // {
          //   bit_pin1=1;
          // }
          // int analog2=analogRead(pin2);
          // uint8_t bit_pin2=0;
          // if(analog2>437)
          // {
          //   bit_pin2=1;
          // }
          // int analog3=analogRead(pin3);
          // uint8_t bit1=0;
          // if(analog3>437)
          // {
          //   bit1=1;
          // }
          // int analog4=analogRead(pin4);
          // uint8_t bit2=0;
          // if(analog4>437)
          // {
          //   bit2=1;
          // }
          // uint8_t bit3=digitalRead(pin5);
          // int data_3bit=bit1+2*bit2+2*2*bit3;
          // //接收信号
          // uint8_t bit1=digitalRead(pin3);
          // int analog4=analogRead(pin4);
          // uint8_t bit2=0;
          // if(analog4>400)
          // {
          //   bit2=1;
          // }
          // uint8_t bit3=digitalRead(pin5);
          // int data_3bit=bit1+2*bit2+2*2*bit3;
         int index=0;
         while (Serial.available() > 0) {  // 检查是否有数据
            if (index < 100) {        // 确保不超过缓冲区大小
              buffer[index++] = Serial.read();  // 读取一个字节并存储
            } else {
              break;  // 缓冲区已满
            }
          }
          for(int i=0;i<index;i++)
          {
            if(buffer[i]==0xaa)
            {
              if(buffer[i+2]==0xbb)
              {
                uart_data=buffer[i+1];
              }
            }
          }

          uint8_t bit_pin1=uart_data&0x01;
          uint8_t bit_pin2=(uart_data>>1)&0x01;
          uint8_t bit1=(uart_data>>2)&0x01;
          uint8_t bit2=(uart_data>>3)&0x01;
          uint8_t bit3=(uart_data>>4)&0x01;
          int data_3bit=bit1+2*bit2+2*2*bit3;
    /*回归正常循迹模式pin1低pin2低*/
    //不做任何覆盖
    if(bit_pin1==0 && bit_pin2==0)
    {
      xyz_change[0]=0;
      xyz_change[2]=0;
      z_down=0;
    }
    //覆盖循迹机械臂姿态
    else
    {
      //注意第一和第二调节模式在arduino中完全相同
          /*第三调节模式pin1高pin2高*/
          //放左边物块
        if(bit_pin1==1 && bit_pin2==1)
        {
          /*other:不动 1:down 5:up 3:back 4:forward 0:重来*/
          if(data_3bit==0)//0:重来
          {
            xyz_change[0]=0;
            xyz_change[2]=0;
          }
          if(data_3bit==1)//1:down
          {
            xyz_change[2]-=0.01;
          }
          if(data_3bit==5)//5:up
          {
            xyz_change[2]+=0.01;
          }
          if(data_3bit==3)//3:back
          {
            xyz_change[0]-=0.01;
          }
          if(data_3bit==4)//4:forward
          {
            xyz_change[0]+=0.01;
          }
          target_xyz[0]=105+xyz_change[0];
          target_xyz[1]=0;
          target_xyz[2]=100+xyz_change[2];
          float target_j[6]={0};
          total_ik(target_xyz[0],target_xyz[1],target_xyz[2],&target_j[0],&target_j[1],&target_j[2],100,105);
          target_j[3]=-target_j[1]-target_j[2];//保持末端水平
          theta[0]=target_j[0]*180/pi;
          theta[1]=target_j[1]*180/pi;
          theta[2]=target_j[2]*180/pi;
          theta[3]=target_j[3]*180/pi+45;
          theta[0]=100;//左转机械臂
          theta[5]=75;
          if(data_3bit==7 || data_3bit==6)
          {
                z_down-=0.25;
                if (z_down<=-160)
                {
                  z_down=-160;
                }
                target_xyz[0]=80+xyz_change[0];
                target_xyz[1]=0;
                target_xyz[2]=85+xyz_change[2]+z_down;
                float target_j[6]={0};
                total_ik(target_xyz[0],target_xyz[1],target_xyz[2],&target_j[0],&target_j[1],&target_j[2],100,105);
                target_j[3]=-target_j[1]-target_j[2];//保持末端水平
                theta[0]=target_j[0]*180/pi;
                theta[1]=target_j[1]*180/pi;
                theta[2]=target_j[2]*180/pi;
                theta[3]=target_j[3]*180/pi+15;
                theta[0]=100;//左转机械臂
                theta[5]=75;
          }
          if(data_3bit==2)
          {
            theta[0]=100;//左转机械臂
            theta[1]=-80;
            theta[2]=45;
            theta[3]=65;
            theta[4]=227;
            theta[5]=75;
          }
        }
        /*************************************/
          /*第一调节模式pin1高pin2低*//*第二调节模式pin1低pin2高*/
          //取物块//放物块
        if( (bit_pin1==1 && bit_pin2==0) || (bit_pin1==0 && bit_pin2==1) )
        {
          /*other:不动 1:down 5:up 3:back 4:forward 0:重来*/
          if(data_3bit==0)//0:重来
          {
            xyz_change[0]=0;
            xyz_change[2]=0;
          }
          if(data_3bit==1)//1:down
          {
            xyz_change[2]-=0.01;
          }
          if(data_3bit==5)//5:up
          {
            xyz_change[2]+=0.01;
          }
          if(data_3bit==3)//3:back
          {
            xyz_change[0]-=0.01;
          }
          if(data_3bit==4)//4:forward
          {
            xyz_change[0]+=0.01;
          }
          target_xyz[0]=105+xyz_change[0];
          target_xyz[1]=0;
          target_xyz[2]=100+xyz_change[2];
          float target_j[6]={0};
          total_ik(target_xyz[0],target_xyz[1],target_xyz[2],&target_j[0],&target_j[1],&target_j[2],100,105);
          target_j[3]=-target_j[1]-target_j[2];//保持末端水平
          theta[0]=target_j[0]*180/pi;
          theta[1]=target_j[1]*180/pi;
          theta[2]=target_j[2]*180/pi;
          theta[3]=target_j[3]*180/pi+45;
          theta[0]=-100;//右转机械臂
          theta[5]=75;
          if(data_3bit==7 || data_3bit==6)
          {
                z_down-=0.25;
                if (z_down<=-160)
                {
                  z_down=-160;
                }
                target_xyz[0]=105+xyz_change[0];
                target_xyz[1]=0;
                target_xyz[2]=100+xyz_change[2]+z_down;
                float target_j[6]={0};
                total_ik(target_xyz[0],target_xyz[1],target_xyz[2],&target_j[0],&target_j[1],&target_j[2],100,105);
                target_j[3]=-target_j[1]-target_j[2];//保持末端水平
                theta[0]=target_j[0]*180/pi;
                theta[1]=target_j[1]*180/pi;
                theta[2]=target_j[2]*180/pi;
                theta[3]=target_j[3]*180/pi+15;
                theta[0]=-100;//右转机械臂
                theta[5]=75;
          }
          if(data_3bit==2)
          {
            theta[0]=-100;//右转机械臂
            theta[1]=-80;
            theta[2]=45;
            theta[3]=65;
            theta[4]=227;
            theta[5]=75;
          }
        }
        /*第一调节模式pin1高pin2低*///end /*第二调节模式pin1低pin2高*///end
        /*************************************/
    }
     
 //最高命令控制夹爪
 if(data_3bit==0&&bit_pin2==0&&bit_pin1==0)//该情况为巡线
  {
    theta[5]=75*2;
  }
  if(data_3bit==7&&bit_pin1==1 && bit_pin2==0)//该情况为取物块ok
  {
    theta[5]=75*2;
  }
  if(data_3bit==2&&bit_pin1==1 && bit_pin2==0)//该情况 #(二货)2表示除了yaw,其他轴还原为循迹(怕磕到设备),这段时间也可以调整车的朝向
  {
    theta[5]=75*2;
  }
  if(bit_pin1==0 && bit_pin2==1)//该情况为放物块
  {
    /*第二调节模式pin1低pin2高*/
          //放物块
    theta[5]=75*2;
    //当放物块ok时
    if(data_3bit==7)//该情况为放物块ok
    {
      theta[5]=75;
    }
  }
  if(bit_pin1==1 && bit_pin2==1)//该情况为放物块
  {
    /*第三调节模式pin1高pin2高*/
          //放物块
    theta[5]=75*2;
    //当放物块ok时
    if(data_3bit==7)//该情况为放物块ok
    {
      theta[5]=75;
    }
  }
  

 /***********************************下面为发送给舵机执行(别动)*******************************************/
 int user_cmd[6]={270/2+theta[0],theta[1]+180,theta[2],theta[3]+85,theta[4],theta[5]};
 /*******************************************************/
 if(uart_data!=0xcc && uart_data!=0xcd && uart_data!=0xce)
 {
  task_count=0;//不写字持续归零
 }

/********************************************************/
if(uart_data==0xcc)
{
  if (task_count < 1500) {
    user_cmd[0] = 150;
    user_cmd[1] = 115;
    user_cmd[2] = -61;
    user_cmd[3] = 145.36;
    user_cmd[4] = 180;
    user_cmd[5] = 0;
} 
else if (task_count >= 1500 && task_count < 3000) {
    // 在1500到3000之间进行插值
    float t = (task_count - 1500) / 1500.0;  // 归一化进度
    user_cmd[0] = (1 - t) * 150 + t * 145;  // 线性插值
    user_cmd[1] = 115;  // 保持不变
    user_cmd[2] = -61;  // 保持不变
    user_cmd[3] = (1 - t) * 145.36 + t * 146.36; // 线性插值
    user_cmd[4] = 180;   // 保持不变
    user_cmd[5] = 0;  // 保持不变
}
else if (task_count >= 3000 && task_count < 4500) {
    // 在3000到4500之间进行插值
    float t = (task_count - 3000) / 1500.0;  // 归一化进度
    user_cmd[0] = (1 - t) * 145 + t * 125;  // 线性插值
    user_cmd[1] = 118;  // 保持不变
    user_cmd[2] = -61;  // 保持不变
    user_cmd[3] = (1 - t) * 146.36 + t * 150.36;  // 线性插值
    user_cmd[4] = 180;    // 保持不变
    user_cmd[5] = 0;   // 保持不变
}

// 竖着
else if (task_count >= 4500 && task_count < 5000) {
    user_cmd[0] = 140;
    user_cmd[1] = 95;
    user_cmd[2] = -61;
    user_cmd[3] = 155.36;
    user_cmd[4] = 180;
    user_cmd[5] = 0;
} 

else if (task_count >= 5000 && task_count < 5500) {
    user_cmd[0] = 135;
    user_cmd[1] = 130;
    user_cmd[2] = -81;
    user_cmd[3] = 100.36;
    user_cmd[4] = 180;
    user_cmd[5] = 0;
} 

else if (task_count >= 5500 && task_count < 6000) {
    user_cmd[0] = 140;
    user_cmd[1] = 135;
    user_cmd[2] = -81;
    user_cmd[3] = 100.36;
    user_cmd[4] = 180;
    user_cmd[5] = 0;
} 

else if (task_count >= 6000 && task_count < 6500) {
    // 在6000到6500之间进行插值
    float t = (task_count - 6000) / 500.0;  // 归一化进度
    user_cmd[0] = 135;  // 保持不变
    user_cmd[1] = (1 - t) * 115 + t * 135;  // 线性插值
    user_cmd[2] = (1 - t) * (-81) + t * (-61);  // 线性插值
    user_cmd[3] = (1 - t) * 145.36 + t * 170.36; // 线性插值
    user_cmd[4] = 180;   // 保持不变
    user_cmd[5] = 0;  // 保持不变
}
user_cmd[4]=user_cmd[4]+90;
}
/******************************************************************/

if(uart_data==0xcd)
{
  if (task_count < 1500) {
    user_cmd[0] = 150;
    user_cmd[1] = 115;
    user_cmd[2] = -61;
    user_cmd[3] = 145.36;
    user_cmd[4] = 180;
    user_cmd[5] = 0;
} 
else if (task_count >= 1500 && task_count < 3000) {
    // 在1500到3000之间进行插值
    float t = (task_count - 1500) / 1500.0;  // 归一化进度
    user_cmd[0] = (1 - t) * 150 + t * 145;  // 线性插值
    user_cmd[1] =115;  // 线性插值
    user_cmd[2] = -61;  // 保持不变
    user_cmd[3] = (1 - t) * 145.36 + t * 146.36; // 线性插值
    user_cmd[4] = 180;   // 保持不变
    user_cmd[5] = 0;  // 保持不变
}
else if (task_count >= 3000 && task_count < 4500) {
    // 在3000到4500之间进行插值
    float t = (task_count - 3000) / 1500.0;  // 归一化进度
    user_cmd[0] = (1 - t) * 145 + t * 135;  // 线性插值
    user_cmd[1] = 115;  // 线性插值
    user_cmd[2] = -61;  // 保持不变
    user_cmd[3] = (1 - t) * 146.36 + t * 150.36;  // 线性插值
    user_cmd[4] = 180;    // 保持不变
    user_cmd[5] = 0;   // 保持不变
}

//竖着
else if (task_count >= 4500 && task_count < 5000) {
    user_cmd[0] = 145;
    user_cmd[1] = 115;
    user_cmd[2] = -61;
    user_cmd[3] = 145.36;
    user_cmd[4] = 180;
    user_cmd[5] = 0;
} 
else if (task_count >= 6000 && task_count < 6500) {
    // 在6000到7500之间进行插值
    float t = (task_count - 6000) / 1500.0;  // 归一化进度
    user_cmd[0] = (1 - t) * 145 + t * 145;  // 保持不变
    user_cmd[1] = (1 - t) * 115 + t * 115;  // 保持不变
    user_cmd[2] = -61;  // 保持不变
    user_cmd[3] = (1 - t) * 145.36 + t * 170.36; // 线性插值
    user_cmd[4] = 180;   // 保持不变
    user_cmd[5] = 0;  // 保持不变
}
else if (task_count >= 7500 && task_count < 13000) {
    // 在7500到9000之间进行插值
    float t = (task_count - 7500) / 5500.0;  // 归一化进度
    user_cmd[0] = (1 - t) * 145 + t * 145;  // 保持不变
    user_cmd[1] = (1 - t) * 115 + t * 115;  // 保持不变
    user_cmd[2] = (1 - t) * (-61) + t * 15;  // 线性插值
    user_cmd[3] = (1 - t) * 160.36 + t * 165.36;  // 线性插值
    user_cmd[4] = 180;    // 保持不变
    user_cmd[5] = 0;   // 保持不变
}
user_cmd[4]=user_cmd[4]+90;
}
if(uart_data==0xce)
{
   // 高位姿
    if (task_count < 1500) {
        user_cmd[0] = 150;
        user_cmd[1] = 115;
        user_cmd[2] = -61;
        user_cmd[3] = 145.36;
        user_cmd[4] = 180;
        user_cmd[5] = 0;
    } 
    else if (task_count >= 1500 && task_count < 3000) {
        // 在1500到3000之间进行插值
        float t = (task_count - 1500) / 1500.0;  // 归一化进度
        user_cmd[0] = (1 - t) * 150 + t * 145;  // 线性插值
        user_cmd[1] = 115;  // 保持不变
        user_cmd[2] = -61;  // 保持不变
        user_cmd[3] = (1 - t) * 145.36 + t * 146.36; // 线性插值
        user_cmd[4] = 180;   // 保持不变
        user_cmd[5] = 0;  // 保持不变
    } 
    else if (task_count >= 3000 && task_count < 4500) {
        // 在3000到4500之间进行插值
        float t = (task_count - 3000) / 1500.0;  // 归一化进度
        user_cmd[0] = (1 - t) * 145 + t * 125;  // 线性插值
        user_cmd[1] = 115;  // 保持不变
        user_cmd[2] = -61;  // 保持不变
        user_cmd[3] = (1 - t) * 146.36 + t * 150.36;  // 线性插值
        user_cmd[4] = 180;    // 保持不变
        user_cmd[5] = 0;   // 保持不变
    }

    // 竖着
    // 4500 到 5000
    else if (task_count >= 4500 && task_count < 5000) {
        float t = (task_count - 4500) / 500.0;  // 归一化进度
        user_cmd[0] = (1 - t) * 150 + t * 150;  // 保持不变
        user_cmd[1] = (1 - t) * 105 + t * 135;  // 线性插值
        user_cmd[2] = -61;  // 保持不变
        user_cmd[3] = (1 - t) * 145.36 + t * 90.36;  // 线性插值
        user_cmd[4] = 180;   // 保持不变
        user_cmd[5] = 0;  // 保持不变
    } 

    // 5000 到 6000
    else if (task_count >= 5000 && task_count < 6000) {
        float t = (task_count - 5000) / 1000.0;  // 归一化进度
        user_cmd[0] = (1 - t) * 150 + t * 140;  // 线性插值
        user_cmd[1] = 135;  // 保持不变
        user_cmd[2] = -61;  // 保持不变
        user_cmd[3] = (1 - t) * 90.36 + t * 90.36;  // 保持不变
        user_cmd[4] = 180;   // 保持不变
        user_cmd[5] = 0;  // 保持不变
    } 

    // 6000 到 7500
    else if (task_count >= 6000 && task_count < 7500) {
        float t = (task_count - 6000) / 1500.0;  // 归一化进度
        user_cmd[0] = (1 - t) * 140 + t * 130;  // 线性插值
        user_cmd[1] = 135;  // 保持不变
        user_cmd[2] = -61;  // 保持不变
        user_cmd[3] = 90.36;  // 保持不变
        user_cmd[4] = 180;   // 保持不变
        user_cmd[5] = 0;  // 保持不变
    } 

    // 7500 到 9000
    else if (task_count >= 7500 && task_count < 9000) {
        float t = (task_count - 7500) / 1500.0;  // 归一化进度
        user_cmd[0] = (1 - t) * 130 + t * 128;  // 线性插值
        user_cmd[1] = 135;  // 保持不变
        user_cmd[2] = -61;  // 保持不变
        user_cmd[3] = (1 - t) * 90.36 + t * 85.36;  // 线性插值
        user_cmd[4] = 180;   // 保持不变
        user_cmd[5] = 0;  // 保持不变
    }

// 9000 到 10500
else if (task_count >= 9000 && task_count < 10500) {
user_cmd[0] = 130;
    user_cmd[1] = 95;
    user_cmd[2] = -61;
    user_cmd[3] = 90.36;
    user_cmd[4] = 180;
    user_cmd[5] = 0;

}
user_cmd[4]=user_cmd[4]+90;
}


  myServo[0].write((user_cmd[0]-compensation[0])*sb_ratio);
  myServo[1].write((user_cmd[1]-compensation[1])*sb_ratio);
  myServo[2].write((user_cmd[2]-compensation[2])*sb_ratio);
  myServo[3].write((user_cmd[3]-compensation[3])*sb_ratio);
  myServo[4].write((user_cmd[4]-compensation[4])*sb_ratio);
  myServo[5].write((user_cmd[5]-compensation[5])*sb_ratio);
  task_count++;
  //算上上面超声波延时,总延时12ms
}
/**
 * @brief two dimensional scara
 * @param handcoor 1:right hand 2:left hand
 * @param angles rad -pi~pi
 */
void scara_inverse_kinematics(float x, float y, float L1, float L2, uint8_t handcoor, float angles[2])
{
    if (pow(x, 2) + pow(y, 2) > pow(L1 + L2, 2)) {
        x = (L1 + L2) * cos(atan2(y, x));
        y = (L1 + L2) * sin(atan2(y, x));
    }
    float cos_beta   = (pow(x, 2) + pow(y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2);
    float sin_beta   = 0.0f;
    float temp       = 1 - pow(cos_beta, 2);
    float calc_error = 0.1f;
    if (temp < 0) {
        if (temp > -calc_error) {
            temp = 0;
        } else {
            return;
        }
    }
    //避免解出奇怪值
    if (cos_beta > 1 || cos_beta < -1) { return; }
    if (handcoor == 1) {
        sin_beta = sqrt(temp);
    } else if (handcoor == 2) {
        sin_beta = -sqrt(temp);
    } else {
    }
    angles[1] = atan2(sin_beta, cos_beta);
    angles[0] = atan2(y, x) - atan2(L2 * sin_beta, L1 + L2 * cos_beta);
}

void xyz2xztheta(float x,float y ,float z,float* x_,float* z_,float* theta_)
{
  *x_=sqrt(x*x+y*y);
  *z_=z;
  *theta_=atan2(y,x);
}

void total_ik(float x,float y, float z,float* j1,float* j2,float* j3,int len1,int len2)
{
  float temp_xzth[3]={0};
  float temp_angles[2]={0};
  xyz2xztheta(x,y,z,&temp_xzth[0],&temp_xzth[1],&temp_xzth[2]);
  *j1=temp_xzth[2];
  scara_inverse_kinematics(temp_xzth[0],temp_xzth[1],len1,len2,2,temp_angles);
  *j2=-temp_angles[0];
  *j3=-temp_angles[1];
}

void getLen2_j3bais(float j4,int* len2,float* j3bais)
{
  *len2=sqrt(105*105+245*245-2*105*245*cos(j4));
  *j3bais=acos((105*105+(*len2)*(*len2)-245)/(2*105*(*len2)));
}

/*超声波距离打印,但是需要串口*/
void printPin0State() {
  bool pin0_state = digitalRead(pin0); // 读取pin0的电平状态
  Serial.print(distance);
  Serial.print('cm  ');
  Serial.println(pin0_state); // 打印pin0的电平状态
}
