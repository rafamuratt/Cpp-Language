#include <avr/sleep.h>
#define TX        2                       // D2 USART                                                                                 
#define RX        3                       // D3 USART                                                                                   
#define PIR       8                       // D8 PIR
#define MIC       A3                      // Analog 3 MIC 
#define IDLE_TIME 30000                   // Tempo mínimo entre leituras 900000 = 15min

unsigned long changeTime;                 // conta tempo desde que ligado
int  MIC_status = 0; 
void sleep();
void sensor();
void sendMsgSpec();
void sendMsgStd();


void setup()
{
  cli();
  PCICR  |= 0b00000011;  // enable port B and C for interrupt (PCI)
  PCMSK0 |= 0b00000001;  // PCINT0  related to the pin PB0 pin12 (D8) sensor PIR
  PCMSK1 |= 0b00001000;  // PCINT11 related to the pin PC3 pin26 (ACD3) sensor MIC
  sei();
  Serial.begin(9600);                                                                                // Serial de 9600 bps p/ o Monitor Serial (PC)
  pinMode(PIR, INPUT_PULLUP);
  pinMode(MIC, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  changeTime = millis();
} 


  ISR(PCINT0_vect)     // Wake up Interrupt related to PIR sensor
{
  sleep_disable();
}

 ISR(PCINT1_vect)      // Mic check interrupt
{
  if(digitalRead(PIR) == LOW)
  {
    MIC_status = 1;
  }
  else
  {
    MIC_status = 0;
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
  int printTime = (IDLE_TIME + 1000)/1000;     // variable to print at sleeping message
  Serial.print("\nThe CPU will sleep in ");
  Serial.print(printTime);
  Serial.print("s \n");
  delay(IDLE_TIME + 1000);
  Serial.println("\nSleeping...");
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  sleep_cpu();
  Serial.println("\nThe CPU just woke up!");
  digitalWrite(LED_BUILTIN, HIGH);
}


void sensor()
{                                                                         
    if ((digitalRead(PIR) == LOW))
    {
      delay(6000); 
      if ((millis() - changeTime) >= IDLE_TIME)                                                            
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
