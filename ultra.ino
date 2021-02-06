#define TrigPin A0
#define EchoPin A1

int speedPin = 5;
int dir1=4;
int dir2=3;

unsigned long lastTime;
double Input, Output, Setpoint=10; 
double outMin=-100;
double outMax=250;
double errSum, lastErr;
double kp=-20, ki=0, kd=0;
void setup()
{
  Serial.begin(9600);
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
        pinMode(speedPin,OUTPUT); // 腳位 
        pinMode(dir1,OUTPUT); // 腳位 
        pinMode(dir2,OUTPUT); // 腳位 
}
void loop()
{
   Input=Filter(changshengbo());
  
  if( Input>0 && Input<80)
  {
      Compute();
      motorf(Output);  //zhuo mian sheng bao you mian dian ji
      Serial.println(Output);
  }
  else
  {
      digitalWrite(dir1, LOW);
      digitalWrite(dir2, LOW);
      analogWrite(speedPin,0);
  }
   delay(50);
   
}
float changshengbo()//zuo bian chao sheng bo
{      
        float Value_cm;
  digitalWrite(TrigPin, LOW); //低高低电平发一个短时间脉冲去TrigPin
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  Value_cm = float( pulseIn(EchoPin, HIGH) * 17 )/1000; //将回波时间换算成cm
         //接收到的高电平的时间（us）* 340m/s / 2 = 接收到高电平的时间（us） * 17000 cm / 1000000 us = 接收到高电平的时间 * 17 / 1000  (cm)
        Serial.println(Value_cm);
        return Value_cm;  
}

// 算术平均滤波法
#define FILTER_N 10
float Filter(float value) {
  int i;
  float filter_sum = 0;
  for(i = 0; i < FILTER_N; i++) {
    filter_sum += value;
    delay(1);
  }
  return (float)(filter_sum / FILTER_N);
}

void motorf(int c)        
    {  
     if(c>=0)
       {
          digitalWrite(dir1, HIGH);
          digitalWrite(dir2, LOW);
          analogWrite(speedPin, c);
       }
      else
       {
          digitalWrite(dir2, HIGH);
          digitalWrite(dir1, LOW);
          analogWrite(speedPin, c);
       }    
    }

void Compute()
{
   /*How long since we last calculated*/
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime);

   /*Compute all the working error variables*/
   double error = Setpoint - Input;
   errSum += (error * timeChange);
   double dErr = (error - lastErr) / timeChange;

   /*Compute PID Output*/
   Output = kp * error + ki * errSum + kd * dErr;
   Output = constrain(Output,outMin,outMax);
   /*Remember some variables for next time*/
   lastErr = error;
   lastTime = now;
}
