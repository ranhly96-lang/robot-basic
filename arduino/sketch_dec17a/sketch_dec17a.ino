#define PIN_xj0  A7
#define PIN_xj1  A6
#define PIN_trig  A1
#define PIN_echo  A2

volatile int car_speed;
volatile long systick_ms_xunji;

char cmd_return_tmp[64];

void AI_xunji_moshi() {
  car_speed = 400;
  if (millis() - systick_ms_xunji > 50) {
    systick_ms_xunji = millis();
    if (analogRead(PIN_xj0) > 512 && analogRead(PIN_xj1) < 512) {
      sprintf(cmd_return_tmp, "{#%03dP%04dT0000!#%03dP%04dT0000!#%03dP%04dT0000!#%03dP%04dT0000!}",6,1500+(200 + car_speed),7,1500-0,8,1500+(200 + car_speed),9,1500-0); //组合指令
      Serial.println(cmd_return_tmp); //解析ZMotor4指令

    } else if (analogRead(PIN_xj0) < 512 && analogRead(PIN_xj1) < 512) {
      sprintf(cmd_return_tmp, "{#%03dP%04dT0000!#%03dP%04dT0000!#%03dP%04dT0000!#%03dP%04dT0000!}",6,1500+car_speed,7,1500-car_speed,8,1500+car_speed,9,1500-car_speed); //组合指令
      Serial.println(cmd_return_tmp); //解析ZMotor4指令
    } else if (analogRead(PIN_xj0) < 512 && analogRead(PIN_xj1) > 512) {
      sprintf(cmd_return_tmp, "{#%03dP%04dT0000!#%03dP%04dT0000!#%03dP%04dT0000!#%03dP%04dT0000!}",6,1500+0,7,1500-(200 + car_speed),8,1500+0,9,1500-(200 + car_speed)); //组合指令
      Serial.println(cmd_return_tmp); //解析ZMotor4指令
    }

  }
}

void setup(){
  Serial.begin(115200); //串口初始化

  car_speed = 0;
  systick_ms_xunji = 0;
  delay(3000);
}

void loop(){
  // 循迹 S1
  // 超声波 S3

  //     管脚定义：
  //     循迹（S1）：A1 A0
  //     超声波（S3）：trig-A3 echo-A2
  //     声音（S4） 2
  //     颜色识别（S6） A5 A4
  //
  //     板载按键 D4
  //
  //     手柄模式：
  //     手柄控制车+6个舵机
  //
  //     智能模式
  //     循迹模式-跟随模式
  //     自由避障-循迹避障
  //     声控循迹-声控夹取
  //     定距夹取-颜色识别
  //     循迹定距-循迹识别
  AI_xunji_moshi();

}