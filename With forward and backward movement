#include <Wire.h>

const int addr = 0x68;
int sk=0,flag=1,adj=0,cmd,chk=0;
long long unsigned int loop_timer, count=0;
float A_X,A_Y,A_Z,a,t,G_Y,G_yt,dps,y,a_y,a_y_i;

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
    
    analogWrite(13,0);
    
    if(count%100 == 0)
      {
        y = y/100;
        analogWrite(13,50);
      }
  }
  Serial.println(F("DONE !!"));
  analogWrite(13,50);
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

    if(sk==0 && (abs(int(a_y)) == 0))
    {
        G_yt = -a_y;
        a_y_i = -a_y;
        sk = 1;

        kp =  25.865;                 
        ki =  93.261;                   
        kd =  (0.51525*1)/3;        //(0.52025)/3

        count = 0;
        Serial.println(F("####"));
    } 

    dps = (G_Y-y)/65.5;

    if(dps>500)
      while(1)
      {
          Serial.println(F("!!!!    -------   ERROR    ERROR   ERROR   ERROR   ERROR   ------ !!!!"));
          analogWrite(13,50);
          delay(370);
          analogWrite(13,0);
          delay(300);
      }
    G_yt += ((dps)*0.004);    // LOOP TIME = 4 ms

    t = (G_yt * 0.85 + a_y * 0.15)+0.4;      // COMPLIMENTARY FILTER

//    Serial.println(t);

    E = abs((t-0)/2);

    P = kp * E;
    I = ki * I;
    D = kd *((E-PE)/0.004);
    PE = E;
    O = P + I + D;

    if(E > 4.50)
    {
      adj = -30;
      analogWrite(13,0);
    }
    else
    {
      analogWrite(13,50);
      adj = 50;
    }

    rt_pwm = 100 + (O) - adj ; 
    lt_pwm = 94 + (O) - adj;

    rt_pwm = map(abs(rt_pwm),0,250,25,255);
    lt_pwm = map(abs(lt_pwm),0,244,21,255);
     
    if(t<0 && sk==1)
        {
          digitalWrite(4,LOW);
          digitalWrite(5,HIGH);
          digitalWrite(6,LOW);
          digitalWrite(7,HIGH);

          analogWrite(10,lt_pwm);
          analogWrite(12,rt_pwm);
        }
    else if(t>0 && sk==1)
        {
          digitalWrite(4,HIGH);
          digitalWrite(5,LOW);
          digitalWrite(6,HIGH);
          digitalWrite(7,LOW); 

          analogWrite(10,lt_pwm);
          analogWrite(12,rt_pwm);
        }

    if(Serial.available()>0)
    {
      cmd = Serial.read();

     if(cmd==48)  // MAKES VALUE OF GYRO AND ACC EQUAL // LOW RELIABILITY
        {
          G_yt = a_y_i;
          Serial.println(F("####"));
        }

      if(cmd == 53)
          {
            count = 0;
            Serial.println("S");
          }
    }

     if((cmd==56||cmd==50) &&  (t>10 || t<-5.5)) //
     {
//      Serial.println("INDIA !!");
      if(cmd==56)
        {
        digitalWrite(4,HIGH);
        digitalWrite(5,LOW);
        digitalWrite(6,HIGH);
        digitalWrite(7,LOW);
        analogWrite(10,255);
        analogWrite(12,255);
        }
      else
      {
        digitalWrite(4,LOW);
        digitalWrite(5,HIGH);
        digitalWrite(6,LOW);
        digitalWrite(7,HIGH);
        analogWrite(10,255);
        analogWrite(12,255);   
      }
     }
     else if(cmd == 56 && t<5) //
     {    
      if(t<1.5 && count++%5==0) //
      {
        digitalWrite(4,LOW);
        digitalWrite(5,HIGH);
        digitalWrite(6,LOW);
        digitalWrite(7,HIGH);
        analogWrite(10,180); //200
        analogWrite(12,180); //200
      }
       
      else if(t<2 && count++ % 3 == 0) //
      {
      digitalWrite(4,LOW);
      digitalWrite(5,HIGH);
      digitalWrite(6,LOW);
      digitalWrite(7,HIGH);
        analogWrite(10,80);
        analogWrite(12,80);
      }
    }
    else if(cmd == 50 && t>-5) //
    {
      if(t>-0.8 && count++%5==0) ///
      {
        digitalWrite(4,HIGH);
        digitalWrite(5,LOW);
        digitalWrite(6,HIGH);
        digitalWrite(7,LOW);
          analogWrite(10,240);
          analogWrite(12,240);
      }
      else if(t>-2 && count++%3==0) ///
      {
        digitalWrite(4,HIGH);
        digitalWrite(5,LOW);
        digitalWrite(6,HIGH);
        digitalWrite(7,LOW);
          analogWrite(10,130);
          analogWrite(12,130);
      }
    }
     
    
    while((millis()-loop_timer)<4);
}

