#include <Wire.h>
#include <MPU6050.h>
#include <I2Cdev.h>
const int addr = 0x68;
long long unsigned int loop_timer, count=0;
float A_X,A_Y,A_Z,a,G_X,G_x_ang,dps,x=0,m=0,n=0,a_y;
unsigned long k=0;
boolean sk;
float E=0, P=0, I=0, D=0, O=0, kp=0, ki=0, kd=0, PE=0, lt_pwm=0, rt_pwm=0;

void setup() {
  Wire.begin();
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200);

  // CHANGING SENSITIVITY OF GYRO TO 500 DPS
  Wire.beginTransmission(addr);                                       
  Wire.write(0x1B);                                                   
  Wire.write(0x08);                                                   
  Wire.endTransmission(true); 

 /* Serial.println(F("CALIBRATING GYROSCOPE .....  17secs"));
  while(count++ < 1)
  {
    Wire.beginTransmission(addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(addr,14,true);
    
    m += Wire.read()<<8 | Wire.read();
    n += Wire.read()<<8 | Wire.read();
    Wire.read()<<8 | Wire.read();
    Wire.read()<<8 | Wire.read(); //TEMPERATURE
    x += Wire.read()<<8 | Wire.read();
    Wire.read()<<8 | Wire.read();
    Wire.read()<<8 | Wire.read();

    if(count%2 == 0)
     {
       x /= 2;  //OFFSET FOR X-GYRO
       m /= 2;  //OFFSET FOR X-ACCEL
       n /= 2;  //OFFSET FOR Y-ACCEL
       Serial.println(k);
     }
}*/
  MPU6050 accelgyro(addr);
  accelgyro.setXAccelOffset(-557);
  accelgyro.setYAccelOffset(-1160);
  accelgyro.setZAccelOffset(1513);
  accelgyro.setXGyroOffset(24);
 
  Serial.println(F("DONE"));
  delay(200);

  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);

  count = 0;
}


void loop() 
{
  loop_timer = millis();

  Wire.beginTransmission(addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(addr,14,true);

  A_X = Wire.read()<<8 | Wire.read();
  A_Y = Wire.read()<<8 | Wire.read();
  A_Z = Wire.read()<<8 | Wire.read();
        Wire.read()<<8 | Wire.read();
  G_X = Wire.read()<<8 | Wire.read();
        Wire.read()<<8 | Wire.read();
        Wire.read()<<8 | Wire.read();

  A_X = A_X/10000;
  A_Y = A_Y/10000;
  A_Z = A_Z/10000;

  a = sqrt(pow(A_X,2)+pow(A_Y,2)+pow(A_Z,2));
  a_y = acos(A_Z/a)*57.2958;

  if(sk==LOW && (abs(int(a_y)) == 0))
  {
    G_x_ang = a_y;

    sk = HIGH;

    kp = 21.5;//33.65//43.5
    ki = 0.00;
    kd = 2.5;//2.25;//2.2;//.63//.94//2.25   //0.005

    count = 0;
    Serial.println(F("###############")); 
  }

  dps = (G_X-x)/65.5;

  if(dps>500)
    while(1)
      Serial.println(F("!!!!  ERROR  !!!!"));

  G_x_ang += (dps*0.004);

  E = a_y ;
  Serial.println(E); 

  P = kp*E;
  I = ki*I;
  I = I + E;
  D = kd*((E-PE));
  PE = E;
  O = P + I + D;

  rt_pwm = O;
  lt_pwm = O;

  /*if(abs(E)>14)
    {rt_pwm = lt_pwm = 255;Serial.println("255");}
  else if(abs(E)>5)
    {rt_pwm = lt_pwm = 150;Serial.println("200");}*/

 /* if(abs(E)>15)
  {
    rt_pwm = constrain(rt_pwm,0,200);
    lt_pwm = constrain(lt_pwm,0,200);
  }*/

  analogWrite(10,lt_pwm);
  analogWrite(11,rt_pwm);

  if(G_x_ang < 0 && sk)
  {
   digitalWrite(4,HIGH);
   digitalWrite(5,LOW);
   digitalWrite(6,LOW);
   digitalWrite(7,HIGH);
  }
  else if(G_x_ang > 0 && sk)
  {
    digitalWrite(4,LOW);
    digitalWrite(5,HIGH);
    digitalWrite(6,HIGH);
    digitalWrite(7,LOW); 
  }

  while((millis()-loop_timer) < 4);
}
