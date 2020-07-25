#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>
#include <Servo.h>
#define ESC_pwm D7
#define Servo_pwm D8
#define battery_analog A0
#define Remote_Servo D5
#define Remote_motor D6
#define Buzzer D4
#define Emer D3
#define ADC_Gain 0.0128
#define imu_g 9.80665
#define imu_6050 16384.0
#define imu_6050_g 131.0
Servo ESC; 
Servo SERVO; 
 int tai=0;
int alarm=0;
int sensor_battery_Value,hz_alarm=2000;
int servo_pwm_sent=1500,esc_pwm_sent=1500;
bool rover_brake=0,rover_emergency=1,tickloop=0;
float battery_volt_Value;
float percent_volt_Value;
unsigned long period = 2000; 
unsigned long last_time = 0;
const int MPU_addr=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
long publisher_timer,publisher_timer_battery,publisher_timer_current,publisher_timer_Buzzer,publisher_timer_temp,publisher_timer_brake;
float rover_linear_x,rover_angular_z;
std_msgs::String imu_msg; 
std_msgs::Float32 temp_msg,current_msg;
std_msgs::Int16 battery_msg;
geometry_msgs::Twist cmd_moter;
ros::Publisher imu("Donkey/Arduino/imu", &imu_msg);
ros::Publisher temp("Donkey/Arduino/temp",&temp_msg);
ros::Publisher battery("Donkey/Arduino/battery",&battery_msg);
ros::Publisher current("Donkey/Arduino/current",&current_msg);

void roverCallBack_drive(const geometry_msgs::Twist& cmd_moter)
{
  rover_linear_x = cmd_moter.linear.x  ;
  rover_angular_z = cmd_moter.angular.z ;
}

void roverCallBack_brake(const std_msgs::Bool& cmd_moter)
{
  rover_brake = cmd_moter.data;
}

void roverCallBack_emergency(const std_msgs::Bool& cmd_moter)
{
  rover_emergency = cmd_moter.data;
}


ros::Subscriber <geometry_msgs::Twist> drive("Donkey/Arduino/cmd_vel", roverCallBack_drive);
ros::Subscriber <std_msgs::Bool> brake("Donkey/Arduino/brake", roverCallBack_brake);
ros::Subscriber <std_msgs::Bool> emergency("Donkey/Arduino/emergency", roverCallBack_emergency);
ros::NodeHandle nh;     
                      
void setup()
{
  nh.initNode();
  nh.advertise(imu);
  nh.advertise(temp);
  nh.advertise(battery);
  nh.advertise(current);
  nh.subscribe(drive);
  nh.subscribe(brake);
  nh.subscribe(emergency);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  
  Wire.write(0);
  Wire.endTransmission(true);

  //////senser  input output//////
   pinMode(Buzzer, OUTPUT);
   pinMode(Remote_Servo, INPUT);
   pinMode(Remote_motor, INPUT);
   SERVO.attach(Servo_pwm,1000,2000);
   ESC.attach(ESC_pwm,1000,2000);
}
void loop()
{
  battery_check();
  readimu();
  motordrive();
  nh.spinOnce();
  
}
void readimu(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  Tmp=Wire.read()<<8|Wire.read();
  GyX=Wire.read()<<8|Wire.read();
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();
  String AX = String(AcX/imu_6050*imu_g);
  String AY = String(AcY/imu_6050*imu_g);
  String AZ = String(AcZ/imu_6050*imu_g);
  String GX = String(GyX/imu_6050_g);
  String GY = String(GyY/imu_6050_g);
  String GZ = String(pulseIn(ESC_pwm, HIGH, 25000));
  String TmpC = String(Tmp);
  String data = "AXAYAZgXgYgZ," + AX + ","+ AY + "," + AZ + "," + GX + "," + GY + "," + GZ + ",G" ;
  int length = data.indexOf("G") +2;
  char data_final[length+1];
  data.toCharArray(data_final, length+1);
  if (millis() > publisher_timer) {
    imu_msg.data = data_final;
    imu.publish(&imu_msg);
    temp_msg.data = Tmp;
    temp.publish(&temp_msg);
    publisher_timer = millis() + 10;
    
  }
  if (millis() > publisher_timer_temp) {
    temp_msg.data = Tmp;
    temp.publish(&temp_msg);
    publisher_timer_temp = millis() + 1000
    ;
    
  }
}

void battery_check(){
  sensor_battery_Value = analogRead(battery_analog);
  battery_volt_Value=sensor_battery_Value*ADC_Gain;
  percent_volt_Value=(battery_volt_Value/0.024)-437.5;
   if(percent_volt_Value>30){
    alarm=0;
    digitalWrite(Buzzer,LOW);
    }

  else if(percent_volt_Value<10){
    alarm=1;
    hz_alarm=100;
  }
  else if(percent_volt_Value<20){
    alarm=1;
    hz_alarm=300;
  }
  else if(percent_volt_Value<20){
    alarm=1;
    hz_alarm=500;
  }
 
 if (millis() > publisher_timer_Buzzer&&alarm==1) {
    digitalWrite(Buzzer,!digitalRead(Buzzer));
     publisher_timer_Buzzer = millis() +hz_alarm ;
 }
 
 if (millis() > publisher_timer_battery) {
      battery_msg.data = percent_volt_Value;
      battery.publish(&battery_msg);
      publisher_timer_battery = millis() + 1000;
   }
   
   
   }
   
void current_check(){ 
   if (millis() > publisher_timer_current) {
      current_msg.data = 0;
      current.publish(&current_msg);
      publisher_timer_current = millis() + 1000;
   }
}
//
void Safety(bool i){
  if(i==1){
    digitalWrite(Emer,LOW);
  }
}

void motordrive(){
 int pulseIn_motor=pulseIn(Remote_motor, HIGH, 25000);
 int pulseIn_servo=constrain(pulseIn(Remote_Servo, HIGH, 25000),1010,2000);
 //////////////////mode//////////////////////////////
 //////////remote_control//////
 if(pulseIn_motor<1300){
  esc_pwm_sent=1550;
  servo_pwm_sent=pulseIn_servo;
 }
  //////////ros-cmd-control//////
 else if(pulseIn_motor>1600&&rover_emergency==1&&rover_brake==0){
  esc_pwm_sent=(rover_linear_x*100)+1500;
  servo_pwm_sent=(rover_angular_z/0.00204)+1500;
  
 }
  //////////brake-control//////
 else if(rover_emergency==1&&rover_brake==1){
  if(tickloop==0){
   int pulsein_now=pulseIn(ESC_pwm, HIGH, 25000);
   tickloop=1;
   
   if(pulsein_now>1549){
     esc_pwm_sent=1300;
   }
  else if(pulsein_now<1451){
    esc_pwm_sent=1700;
  }
  publisher_timer_brake=millis();
  }
  if((millis()-publisher_timer_brake)>5000){
    esc_pwm_sent=1500;
  }

  
  
  servo_pwm_sent=1500;
  
 }
 //////////Safety//////
 else if(rover_emergency==0) {
  esc_pwm_sent=1500;
  servo_pwm_sent=1500;
  digitalWrite(Buzzer,HIGH);
 }
 else{
  esc_pwm_sent=1500;
  servo_pwm_sent=pulseIn_servo;
  
 }
 
  SERVO.write(servo_pwm_sent);
  ESC.write(esc_pwm_sent);


}
