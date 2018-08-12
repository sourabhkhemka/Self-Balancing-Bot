#include <Wire.h>
#include <MPU6050.h>
#include <I2Cdev.h>

const int MPU_ADDR=0x68;
long unsigned int enc_count_1=0, enc_count_2=0,loop_timer;
int rpm_1,rpm_2,cmd=0,count=0;
unsigned int fwd_count=0;
long G_X,A_X,A_Y,A_Z;
float dps,set_point,error,prev_error,P=0,I=0,D=0,PWM=0,kp=0,ki=0,kd=0,dps_offset=0;
float accel_vect=0,accel_angle,gyro_angle,set_angle=0;
long long unsigned int timer;
boolean flag;

//USE VOID FUNC RX_BT_CMD() FOR BLUETOOTH COMMUNICATIONS

void setup()
{ 
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);

  Serial.begin(115200);
  
  MPU_INITIALIZE();
  SET_MPU_SENSITIVITY();
  SET_MPU_OFFSET();
  Serial.println(F("DETERMINING SET-POINT..."));
  DYNAMIC_TUNING();
  Serial.println("DONE!!");
}



void loop() 
{
  GET_MPU_DATA();
  
  PID_OUTPUT_CALC();

  kp = 6;//4.2;//3.2
  ki = 0;
  kd = 0;

  BOT_BALANCE();


}

void PID_OUTPUT_CALC()
{
  accel_angle = accel_angle - set_angle;               //USING DYNAMICALLY DETERMINED SET POINT
  accel_angle = abs(0.98*gyro_angle + 0.02*accel_angle*(gyro_angle/abs(gyro_angle)));
  set_point = 5*pow(accel_angle,2);                      
  set_point = set_point * (-gyro_angle/abs(gyro_angle));

  error = (dps-dps_offset) - set_point;
  
  P = kp*error;
  I = ki*error;
  I = I + error;
  D = kd*(error - prev_error);
  prev_error = error;
  
  PWM = abs(P+I+D);
  if(PWM>255)
  PWM = 255;

  Serial.println(accel_angle);
}





void isr_1()
{
  enc_count_1++;
}

void isr_2()
{
  enc_count_2++;
}

void rpm_calc()
{
  enc_count_1 = enc_count_2 = 0;
  timer = millis();

  attachInterrupt(digitalPinToInterrupt(2),isr_1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(3),isr_2,CHANGE);
  while((millis()-timer)<10);  // 0.6667.. PULSE IN 100MS AT 1RPM
  detachInterrupt(digitalPinToInterrupt(2));
  detachInterrupt(digitalPinToInterrupt(3));
 
  rpm_1 = enc_count_1/0.0667;
  rpm_2 = enc_count_2/0.0667;   
}

void MPU_INITIALIZE()
{
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true); 
}

void SET_MPU_SENSITIVITY()
{
  Wire.beginTransmission(MPU_ADDR);                                       
  Wire.write(0x1B);                                                        
  Wire.write(0x08);                                                   
  Wire.endTransmission(true);  
  //GYRO SENS. SET TO NEW VALUE 500 dps
  //ACCEL. SENS. SET TO DEFAULT +/-2g
}

void SET_MPU_OFFSET()
{
  MPU6050 accelgyro(MPU_ADDR);
  accelgyro.setXAccelOffset(-646);
  accelgyro.setYAccelOffset(-1290);
  accelgyro.setZAccelOffset(1532);
  accelgyro.setXGyroOffset(23);
}

void GET_MPU_DATA()
{ 
  loop_timer = millis();
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR,14,true);

  A_X = Wire.read()<<8 | Wire.read();
  A_Y = Wire.read()<<8 | Wire.read();
  A_Z = Wire.read()<<8 | Wire.read();
        Wire.read()<<8 | Wire.read();
  G_X = Wire.read()<<8 | Wire.read();
        Wire.read()<<8 | Wire.read();
        Wire.read()<<8 | Wire.read();

  dps = G_X/65.5;

  accel_vect = sqrt(pow(A_X,2)+pow(A_Y,2)+pow(A_Z,2));
  accel_angle = acos(A_Z/accel_vect)*57.2958;

  if(!flag)
  {
    gyro_angle = accel_angle;
    flag = HIGH;
  }

  gyro_angle += (dps * 0.003); 

  while((millis()-loop_timer) < 3);
}

void BOT_BALANCE()
{
  analogWrite(10,PWM);
  analogWrite(11,PWM);
  
  if(error<0)
  {
   digitalWrite(4,LOW);
   digitalWrite(5,HIGH);
   digitalWrite(6,LOW);
   digitalWrite(7,HIGH); 
  }
  else if(error>0)
  {
   digitalWrite(4,HIGH);
   digitalWrite(5,LOW);
   digitalWrite(6,HIGH);
   digitalWrite(7,LOW);
  }
}

void RX_BT_CMD()
{
  if(Serial.available()>0)
    cmd = Serial.read();
}

void DYNAMIC_TUNING()
{
  count=0;
  while(count++<2000)
  {
    GET_MPU_DATA();
    set_angle += (accel_angle * (gyro_angle/abs(gyro_angle)) );
    dps_offset += dps; 
  }
  set_angle /= 2000;
  dps_offset /= 2000;
}
