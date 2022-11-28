#include <Arduino.h>
#define DREAPTA 1
#define STANGA 2

const int motorStang1 = 6;  //fata
const int motorStang2 = 7;  //spate
const int motorDrept1 = 4;  //fata
const int motorDrept2 = 5;  //spate
const int motorStangPWM = 9;
const int motorDreptPWM = 10;
const int CALIBRATION_TIME = 800; // in milisecunde
const int CALIBRATION_SPEED = 100;
int DISTANTA_PONDERE = 23;         //distanta intre sensori in milimetrii
int TARGET = 2*DISTANTA_PONDERE;
int sensors[5] = {A0, A1, A2, A3, A4};
int sensorValue[5];
int max_sensors[5];

int min_sensors[5];         
int speedPercent = 75;  //80
int maxSpeed =(speedPercent*255)/100;
int lineDetected = 0, sumError = 0, ct = 0;
float totalError = 0, lastError = 0, lastTime = 0, linePosition = 0;
float KS = 0;   // constanta de viteza 


float KP = 4.75;   //5   5,4   5.5             5.4          
float KD = 25.5;  //                               38.5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        ;    //40   39   36  pe 75    36.5
float KI = 0;


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void initPins()
{
  int i;
  pinMode(motorDrept1, OUTPUT);
  pinMode(motorDrept2, OUTPUT);
  pinMode(motorStang1, OUTPUT);
  pinMode(motorStang2, OUTPUT);
  pinMode(motorDreptPWM, OUTPUT);
  pinMode(motorStangPWM, OUTPUT);

  for(i = 0; i<5; i++)
  {
    pinMode(sensors[i], INPUT);
  }

  for(i = 0; i<5; i++)
  {
    max_sensors[i] = 0;
    min_sensors[i] = 1024;
  }

}

void motorSpeed(int speedDrept, int speedStang)
{
  speedDrept = constrain(speedDrept, -255, 255);
  speedStang = constrain(speedStang, -255, 255);

  if(speedDrept < 0)
  {
    digitalWrite(motorDrept1, LOW);
    digitalWrite(motorDrept2, HIGH);
    //Serial.println(F("BACK DR"));
  }
  else
  {
    digitalWrite(motorDrept1, HIGH);
    digitalWrite(motorDrept2, LOW);
    //Serial.println(F("FORW DR"));
  }
  if(speedStang < 0)
  {
    digitalWrite(motorStang2, HIGH);
    digitalWrite(motorStang1, LOW);
    //Serial.println(F("BACK ST"));
  }
  else
  {
    digitalWrite(motorStang1, HIGH);
    digitalWrite(motorStang2, LOW);
    //Serial.println(F("FORW ST"));
  }

  analogWrite(motorDreptPWM, abs(speedDrept));
  analogWrite(motorStangPWM, abs(speedStang));

}

void calibrate(int direction)
{

  int i, value, ms;
  
  ms = millis();

  while(millis() - ms < CALIBRATION_TIME) 
  {
    if(direction == DREAPTA)
      motorSpeed(-CALIBRATION_SPEED, CALIBRATION_SPEED);
    if(direction == STANGA)
      motorSpeed(CALIBRATION_SPEED, -CALIBRATION_SPEED);

    for(i = 0; i<5; i++)
    {
      value = analogRead(sensors[i]);
      if(value > max_sensors[i])
      {
        max_sensors[i] = value;
      }
      if(value < min_sensors[i])
      {
        min_sensors[i] = value;
      }
    }
  }
  motorSpeed(0,0);

}

int ComputePID(float error, float kp, float kd, float ki)
{
  float proportional, derivative, integral, correction;
  
  
  proportional = error;
  derivative = error - lastError;
  integral += error;

  
    //Serial.print(derivative, DEC); Serial.print('\n');
    
  correction = kp*proportional + kd*derivative + ki*integral;
  
  lastError = error;


  return floor(correction);
}

void updatePosition()
{
  float numarator, numitor, x;
  lineDetected = 0;
  numarator = 0.0;
  numitor = 0.0;
  for(int i = 0; i<5; i++)
  { 
    x = analogRead(sensors[i]);

    x = mapFloat(x, min_sensors[i], max_sensors[i], 1000, 0);  //e int

    sensorValue[i] = x;

    if(x > 800)
    {
      lineDetected++;
    }
    
    numarator = numarator + x * DISTANTA_PONDERE * i;
    numitor = numitor + x;
  }
  if(lineDetected > 0)
  {
    linePosition = numarator / numitor;
  }
 
}
  
  
void mainLoop()
{
  
  int mDrept, mStang, speedRed, correction, error;
  updatePosition();
  
  error = linePosition - TARGET;
  
  if(lineDetected > 1)
  {
    if(error > 0) error = 46, linePosition = 92;
    else if(error < 0) error = -46, linePosition = -92;
    correction = ComputePID(error, KP, KD, KI);
  }
  else
  {
    correction = ComputePID(error, KP, KD, KI);
  }
 
   
   
   //Serial.println(error, DEC);
 
     if(correction > 0)
      {
                //viraj la dreapta => MOTORU DREPT AFECTAT
             mDrept = maxSpeed - correction - speedRed;
             mStang = maxSpeed - speedRed;
      }
      else
      {
                //viraj la stanga => MOTORU STANG AFECTAT
             correction = -correction;
                                                        
             mDrept = maxSpeed - speedRed;
             mStang = maxSpeed - correction - speedRed;        
      }  
    
        
     motorSpeed(mDrept, mStang); 
           

   
  }


void setup() {
  initPins();
  Serial.begin(9600);
  calibrate(DREAPTA);
  calibrate(STANGA);
  calibrate(STANGA);
  calibrate(DREAPTA);
  motorSpeed(0,0);
  delay(1000);

}

void testPID()
{
  float k_p, k_d, k_i;
  while(true)
  {
    while(Serial.available() < 3){}
    k_p = Serial.parseFloat();
    k_d = Serial.parseFloat();
    k_i = Serial.parseFloat();

    
    Serial.print(k_p, DEC); Serial.print(" ");
    Serial.print(k_d, DEC); Serial.print(" ");
    Serial.print(k_i, DEC); Serial.print('\n');
    
  

    KP = k_p;
    KD = k_d;
    KI = k_i;

    
 
    while (true)
    {
      mainLoop();
      if(Serial.read() == 's')
      {
        motorSpeed(0,0);
        break;
      }
      delay(12);

    }
  }
}

void loop() {

  //s
  //testPID();

  
  while(true)
  {
    mainLoop();
    delay(12);
  }

}
