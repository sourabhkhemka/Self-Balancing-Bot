
#include <Wire.h>

const int addr = 0x68;
int count = 0,sk=0,flag=1;
long long unsigned int loop_timer;
float A_X,A_Y,A_Z,a,t,G_Y,G_yt,dps,y,a_y;

float E, P, I, D, O, kp=0, ki=0, kd=0, PE=0, lt_pwm, rt_pwm;

void setup() 
{ 
  //Initialising
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

  // CALIBRATING GYRO TO GET THE OFFSET VALUE
  Serial.println(F("CALIBRATING GYROSCOPE ....."));
  while(count++ < 300)                           // USING 3000 SENSOR DATA
  {
    // REQUESTING THE SENSOR VALUES FROM MPU-6050
    Wire.beginTransmission(addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(addr,14,true);             // 14 BYTES TO BE RECIEVED
    
    Wire.read()<<8 | Wire.read();               // COMBINING MSB AND LSB TO GET COMPLETE SENSOR DATA
    Wire.read()<<8 | Wire.read();
    Wire.read()<<8 | Wire.read();
    Wire.read()<<8 | Wire.read();
    Wire.read()<<8 | Wire.read();
    y += Wire.read()<<8 | Wire.read();         // STORING REQUIRED SENSOR DATA IN A VARIABLE
    Wire.read()<<8 | Wire.read();

    analogWrite(13,0);                        // LED BLINKS FAST AND WITH DIM LIGHT WHILE CALIBRATING
    
    if(count%100 == 0)
      {
        y = y/100;                            // TAKING MEAN OF EVERY 100 VALUES TO PREVENT OVERFLOW IN THE VARIABLE y
        analogWrite(13,50);                   // LED BLINKS FAST AND WITH DIM LIGHT WHILE CALIBRATING 
      }
  }
  Serial.println(F("DONE !!"));
  analogWrite(13,50);                        // GLOW LED ATTACHED TO digitalpiin 13 ONES CALIBRATED
  
  delay(200);

  pinMode(4,OUTPUT);                         // SETTING THE DIRECTION PINS AS OUTPUT
  pinMode(5,OUTPUT);  
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);

  count = 0;
}

void loop()
{
  loop_timer = millis();                   // STORING THE STARTING TIME OF LOOP IN loop_timer AND USING while(millis() - loop_timer > 4)
                                           // AT THE END TO SET THE EXECUTION TIME OF LOOP TO 4 MILLI SECONDS

  // REQUESTING SENSOR DATA
  Wire.beginTransmission(addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(addr,14,true);

  // STORING REQUIRED SENSOR DATA IN VARIABLES
   
    A_X = Wire.read()<<8 | Wire.read();       // ACC. (X AXIS)
    A_Y = Wire.read()<<8 | Wire.read();       // ACC. (Y AXIS)
    A_Z = Wire.read()<<8 | Wire.read();       // ACC. (Z AXIS)
          Wire.read()<<8 | Wire.read();       // TEMP.
          Wire.read()<<8 | Wire.read();       // GYRO (X AXIS)
    G_Y = Wire.read()<<8 | Wire.read();       // GYRO (Y AXIS)
          Wire.read()<<8 | Wire.read();       // GYRO (Z AXIS)
  
   // DIVIDED THE RAW ACCELEROMETER DATA BY SOME RANDOM LARGE VALUE TO LIMIT THE SIZE OF VALUES TO THE RANGE OF VARIABLE TYPE

    A_X = A_X/10000;                           
    A_Y = A_Y/10000;
    A_Z = A_Z/10000;
    
    a = sqrt(pow(A_X,2) + pow(A_Y,2) + pow(A_Z,2)); // ACCELERATION VECTOR
    a_y = acos(A_Z/a) * 57.2958;                    // ANGLE OF BOT wrt VERTICAL (ANGLE b/w Z-AXIS AND ACCELERATION VECTOR)

    if(sk==0 && (abs(int(a_y)) == 0))              // START THE ROBOT WHEN THE ANGLE OF INCLINATION OF BOT IS LESS THAN 1 deg
    {
        G_yt = -a_y;                                // INITIALLY COPYING ACCELEROMETER VALUE IN GYRO AND THEN CALCULATING ANGLE USING
        sk = 1;                                     // ANGULAR VELOCITY OF BOT                              
      
        // SETTING  VALUES FOR kp, ki AND kd
        kp = 25.865;                    //28.765          
        ki = 93.261;                   //100.575          /* ## TWO SET OF VALUES ## COMMENTED ONES ARE FOR WHEN MAPPING(map())                         
        kd = (0.51525*1)/3;           //(0.57025)/3                                                                NOT DONE ## */

        count = 0;                                  // SETTING count TO 0 // TO MAKE IT AVAILABLE IF REQUIRED AT SOME-TIME
        Serial.println(F("##########################################"));    // TO CONFORM IF INSTRUCTION TO START THE BOT EXECUTED
    } 

    dps = (G_Y-y)/65.5;                             // GYRO SENSITIVITY IS SET TO 500 dps
        
    if(dps>500)                                    // IF dps GREATER THAN 500(WHICH IS THE GYRO SENSITIVITY)
      while(1)
      {
          Serial.println(F("!!ERROR!!"));                // 1. ERROR MESSAGE AND void loop HANGED 
          analogWrite(13,50);                            
          delay(370);                                    // 2. LED AT PIN 13 START BLINKING AND void loop HANGED
          analogWrite(13,0);
          delay(300);
      }
  
    G_yt += ((dps)*0.004);                          // CALCULATING ANGULAR VELOCITY USING EQUATIONS OF MOTIO // LOOP TIME = 4 ms

    t = G_yt * 0.85 + a_y * 0.15;                   // COMPLIMENTARY FILTER

    Serial.println(t);

    E = abs((t)/2);                                // MAY ADD SOME VALUE TO t TO COMPENSATE ERROR IN MPU PLACEMENT OR IN CASE BIASED
                                                  //  O/p FOR +ve AND -ve ROTATION 
  
    // CALCULATING P , I  AND  D
    P = kp * E;
    I = ki * I;
    D = kd *((E-PE)/0.004);
    PE = E;
    O = P + I + D;

    // SETTING PWM 
    rt_pwm = 100 + (O);                             // SETTING VARIABLE CONTANTS FOR THE TWO MOTORS IN ORDER TO REMOVE ERRORS IN O/P VOLTAGE
    lt_pwm = 94 + (O);                             //  IN THE MOTOR DRIVER

  
    rt_pwm = map(abs(rt_pwm),0,250,25,255);        // MOTOR STOPS TO ROTATE AT A CERTAIN VALUE OF pwm, THAT VALUE OF pwm IS CALLED
    lt_pwm = map(abs(lt_pwm),0,244,21,255);       // MOTOR MINIMUM. THE MIN. VALUE OF abs(pwm) IS MAPPED TO MOTOR MIN. AND MAX VALUE
                                                 //  (approx) IS MAPPED TO 255(max possible pwm signal)
  
  
   // TO CONTROL THE DIRECTION OF MOTORS BASED ON ERROR (E wrt THE SET-POINT i-e 0)
    if(t<0 && sk==1)
        {
          digitalWrite(4,LOW);
          digitalWrite(5,HIGH);
          digitalWrite(6,HIGH);
          digitalWrite(7,LOW);

          analogWrite(10,lt_pwm);                  // PWM SIGNAL FROM DIGITAL PIN 10 IS FOR MOTOR CONNECTED TO LEFT H-BRIDGE
          analogWrite(13,rt_pwm);                 // PWM SIGNAL FORM DIGITAL PIN 13 IS FOR MOTOR CONNECTED TO RIGHT H-BRIDGE
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
    if(count%3000 == 0)                       // copying accelerometer value in gyro every 3000 loop cycles to eliminate drifts
        G_Y = a_y;                            // in gyro readings
    
    */ /* > !! NOT USED BECAUSSE ACCELEROMETER READING DOESN'T CONTAIN THE DIRECTION OF ROTATION(+/-) THUS WILL RESULT IN EVEN POOR 
          PERFORMANCE*/
  
    while((millis()-loop_timer)<4);           // TO CONTROL void loop() EXECUTION TIME
}


