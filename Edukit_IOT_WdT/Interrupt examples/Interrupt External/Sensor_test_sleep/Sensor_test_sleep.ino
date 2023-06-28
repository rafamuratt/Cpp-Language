#include <avr/sleep.h>
#define TX  2                                                                                      // Serial TX 
#define RX  3                                                                                      // Serial RX
#define PIR 2
#define MIC 3


unsigned long changeTime;                                                                            // conta tempo desde que ligado
int  MIC_status = 0; 
void sleep();
void sensor();
void sendMsgSpec();
void sendMsgStd();
void PIR_Interrupt();
void MIC_Interrupt();


void setup()
{
  Serial.begin(9600);                                                                                // Serial de 9600 bps p/ o Monitor Serial (PC)
  pinMode(PIR, INPUT_PULLUP);
  pinMode(MIC, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  changeTime = millis();
} 


void PIR_Interrupt()
{
  sleep_disable();
  detachInterrupt(0);
}


void MIC_Interrupt()
{   
  int PIR_Sensor = digitalRead(PIR);                                                                // porta 2   
  if(PIR_Sensor == LOW)
  {
    MIC_status = 1;
  }
  else
  {
    MIC_status = 0;
    detachInterrupt(1); 
  }    
}


void loop()
{
  sleep();
  while (true)
  {
    sensor();
  } 
} 


void sleep()
{
  Serial.println("\nThe CPU will sleep in 5s");
  delay(5000);
  Serial.println("\nSleeping...");
  sleep_enable();
  attachInterrupt(0, PIR_Interrupt, LOW);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  sleep_cpu();
  Serial.println("\nThe CPU just woke up!");
  digitalWrite(LED_BUILTIN, HIGH);
}


void sensor()
{      
    int PIR_Sensor = digitalRead(PIR);                                                                  // porta 2   
    if ((PIR_Sensor == LOW))
    {
      attachInterrupt(1, MIC_Interrupt, LOW);                                                           // porta 3
      delay(6000);  
      if ((millis() - changeTime) >= 30000)                                                             // 15 min = 900000
      {        
        if (MIC_status == 1)                                                      
        {
          sendMsgSpec();
        }
        else
        {
          sendMsgStd();
        }
      }
      else
      {
        Serial.println(MIC_status);
      }
    }
    else
    {
      Serial.println("\nNo action");
      loop();
    }
}


void sendMsgSpec()
{
  Serial.println("\nAcesso especial");
  delay(6000);
  Serial.println("\nAcesso especial enviado");
  delay(6000);
  MIC_status = 0;
  detachInterrupt(1);
  changeTime = millis();
}


void sendMsgStd()
{
  Serial.println("\nAcesso comum");
  delay(6000);
  Serial.println("\nAcesso comum enviado");
  delay(6000);
  changeTime = millis();
}
