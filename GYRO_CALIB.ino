
#include <Wire.h>

const int addr = 0x68;
int count = 0,sk=0,flag=1;
long long unsigned int loop_timer;
float A_X,A_Y,A_Z,a,t,G_Y,G_yt,dps,y,a_y;

float E, P, I, D, O, kp=0, ki=0, kd=0, PE=0, lt_pwm, rt_pwm;

void setup() 
{
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

  Serial.println(F("CALIBRATING GYROSCOPE .....  17secs"));
  while(count++ < 2000)
  {
    Wire.beginTransmission(addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(addr,14,true);
    
    Wire.read()<<8 | Wire.read();
    Wire.read()<<8 | Wire.read();
    Wire.read()<<8 | Wire.read();
    Wire.read()<<8 | Wire.read();
    Wire.read()<<8 | Wire.read();
    y += Wire.read()<<8 | Wire.read();
    Wire.read()<<8 | Wire.read();

    if(count%100 == 0)
      {
        y = y/100;
      }
  }
  Serial.println(F("DONE !!"));
  delay(200);

  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);  
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);

  count = 0;
}

void loop()
{
  digitalWrite(2,LOW);

  loop_timer = millis();

  Wire.beginTransmission(addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(addr,14,true);

    A_X = Wire.read()<<8 | Wire.read();       // ACC. (X AXIS)
    A_Y = Wire.read()<<8 | Wire.read();       // ACC. (Y AXIS)
    A_Z = Wire.read()<<8 | Wire.read();       // ACC. (Z AXIS)
          Wire.read()<<8 | Wire.read();       // TEMP.
          Wire.read()<<8 | Wire.read();       // GYRO (X AXIS)
    G_Y = Wire.read()<<8 | Wire.read();       // GYRO (Y AXIS)
          Wire.read()<<8 | Wire.read();       // GYRO (Z AXIS)

    A_X = A_X/10000;                           
    A_Y = A_Y/10000;
    A_Z = A_Z/10000;
    
    a = sqrt(pow(A_X,2) + pow(A_Y,2) + pow(A_Z,2));
    a_y = acos(A_Z/a) * 57.2958;

    if(sk==0 && (abs(a_y) < 1))
    {
        G_yt = -a_y;
        sk = 1;
                  //ku = 240
        kp = 144;          //52.8    //64
        ki = 440;       //74.5    //74.5
        kd = 42.08;       //0.3     //0.3

        count = 0;
        Serial.println(F("##########################################"));
    } 

    dps = (G_Y-y)/65.5;
        
    G_yt += ((dps)*0.004);    // LOOP TIME = 4 ms

    t = G_yt * 0.85 + a_y * 0.15;      // COMPLIMENTARY FILTER

    Serial.println(G_yt);

    E = abs((t+0.3)/2);

    P = kp * E;
    I = ki * I;
    D = kd *((E-PE)/0.004);
    PE = E;
    O = P + I + D;

    rt_pwm = (O); 
    lt_pwm = (O) ;

    if(t<0 && sk==1)
        {
          digitalWrite(4,LOW);
          digitalWrite(5,HIGH);
          digitalWrite(6,HIGH);
          digitalWrite(7,LOW);

          analogWrite(10,lt_pwm);
          analogWrite(13,rt_pwm);
        }
    else if(t>0 && sk==1)
        {
          digitalWrite(4,HIGH);
          digitalWrite(5,LOW);
          digitalWrite(6,LOW);
          digitalWrite(7,HIGH); 

          analogWrite(10,lt_pwm);
          analogWrite(13,rt_pwm);
        }
    /*
    if(count%3000 == 0)
        G_Y = a_y;
    */
    while((millis()-loop_timer)<4);
}


