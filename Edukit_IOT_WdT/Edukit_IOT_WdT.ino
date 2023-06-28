/*
   25/02/2023 V2.5 by rafamuratt

   This firmware makes Arduino sleep to save battery. It only wake up with PIR movement sensor (Interrupt 0 at D2).
   The interrupt 1 (D3) is implemented to check the counter or to send a special message.

   As long the MSG_LIMIT is not over, it will send a Standard Message (ID = 1, or in case of error ID = 2) 
   or Special Message (ID = 4, or in case of error ID = 5) via Sigfox, to update a TagoIO graph.
   
   In case MSG_LIMIT its over, an AlarmMsg will be sent with (ID = 7, or in case of error ID = 8) and the Message services are stopped.
   
   The message content is: battery status (voltage 16 bits, current 16 bits) + an access ID (msgID 8 bits)
 
   Note: Because the HT32SX has physical UART (TX/ RX) connected to Arduino Nano D2 and D3,
   to use the Interruptions 0 and 1 (also located at D2 and D3)a hardware change is required.
   When it's done, there´s no conflict between interruptions and, the UART communication pins with HT32SX
   could be any (eg. here D11 and D12 are being used as TX/RX - SoftwareSerial).
   Also the connector (not the port) D5 is used to activate the interrupt 1 (D3) because the Edukit has a push button (SW) physically connected to it.
   This way, each time SW is pressed (and if IDLE_TIME is not over), the Special Message is manually send, otherwise, the counter status is showed.

   Because of communication issues between Arduino and EduKit,  the functions msgStatus() and orMsg_handler() are implemented to avoid any hang up and 
   send the messages anyways, but with corresponding error ID for tracking.
   But if in case the firmware anyways hangs for more than 80s (COUNTMAX = 10 * 8 seconds), the interrupt-based Watchdog Timer is over (not reseted at next loop). 
   This way, the reset-only Watchdog Timer of 16ms clear the flags and restart the Arduino.

   The Watchdog delay interval patterns are:
   16 ms:     0b000000
   500 ms:    0b000101
   1 second:  0b000110
   2 seconds: 0b000111
   4 seconds: 0b100000
   8 seconds: 0b100001
*/

//=====================================================================================================================================================================================
// Preprocessor & Definitions

#include <SoftwareSerial.h>
#include <Wire.h>
#include <stdlib.h>
#include <avr/wdt.h>                        // To use watchdog timers in AVR
#include <Adafruit_INA219.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define MSG_LIMIT     30                   // Limit to send mensages
#define IDLE_TIME     300000               // Pausa de 5 min, evita o envio de muitas msg
#define COUNTMAX      40                   // Interrupt-based Watchdog Timer expires after about 320 secs 5,33 min (if 8 sec interval is selected).
#define DET_THRESHOLD 7                    // 6, 9 Sensilibilidade para reconhecer a vibração    
#define DET_DURATION  3                    // 3 Tempo da vibração                                                                                
#define RESET         4                    // D4 Reset do HT32SX (Edukit)  60000

//=====================================================================================================================================================================================
// Objects Definition

SoftwareSerial serial_HT(12, 11);          // objeto serial_HT para conectar ao HT32SX:  serial_HT(12, 11) = D12, D11 USART
Adafruit_INA219 ina219(0x40);              // Cria o objeto ina219 (end. 0x40) para leitura dos dados
Adafruit_MPU6050 mpu;                      // Cria o objeto mpu para detecção de vibração

//=====================================================================================================================================================================================
// Global Variables

volatile int counter;                      // Count number of times ISR is called.
int  error_check       = 0;                // Error counter
int  msg_cnt           = 0;                // contagem de mensagens standard enviadas
int  msgID             = 0;
int  pir_sensor        = 0;
int  acc_sensor        = 0;
char PIR               = 2;                // D2
char ACC               = 3;                // D3 ligação paralela com D5 (botão do Edukit)
char error_flag        = 0;                // flag para envio de diferentes estados do msgID
char acc_flag          = 0;                // flag para enviar acc error msg (msgID = 5)
char SW_status         = 0;                // testa estado da chave SW
char region            = 1;                // 1 Europa
char buff[36];                             // Buffer usado para conversão de String.
float voltage_V        = 0.0;
float current_mA       = 0.0;
float voltage_shunt;

//=====================================================================================================================================================================================
// Functions Prototypes

void PIR_Interrupt();
void ACC_Interrupt();
void BT_Interrupt();
void watchdogEnable();
void sleep();
void sensor();
void sendMsgSpec();
void sendMsgStd();
void sendAlarmMsg();
void BT_status();
void error_status();
void reset_();
void msgStatus();
void errorMsg_handler();

//=====================================================================================================================================================================================
// Setup

void setup()
{
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);
  delay(1000);
  digitalWrite(RESET, LOW);
  delay(100);
  pinMode(PIR, INPUT);
  pinMode(ACC, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.begin(9600);                                        // Serial de 9600 bps p/ o Monitor Serial (PC)
  serial_HT.begin(9600);                                     // Serial de 9600 bps p/ o chip HT32SX (Edukit)
  ina219.begin();
  mpu.begin();
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(DET_THRESHOLD);
  mpu.setMotionDetectionDuration(DET_DURATION);              // default = 20
  mpu.setMotionInterrupt(true);                              // true = liga a interrupção baseada no threshold e duração. false desliga ele
  mpu.setInterruptPinLatch(false);                           // true = keep it latched (Will turn off when reinitialized. false = reset after 50us
  mpu.setInterruptPinPolarity(true);                         // true = low when interrupt is active. false = high
  delay(1000);
  reset_();
 
}

//=====================================================================================================================================================================================
// Interruptions

ISR(WDT_vect)                                                // watchdog timer interrupt service routine
{
  counter+=1;
  if (counter < COUNTMAX)
    wdt_reset();                                             // start timer again (still in interrupt-only mode)
    
  else                                                       // then change timer to reset-only mode with short (16 ms) fuse
  {
    Serial.println("###################################");   // print anything, just to track the reset-based Watchdog Timer using the Serial Monitor
    delay(5000);
    MCUSR = 0;                                               // reset flags
                                                             // Put timer in reset-only mode:
    WDTCSR |= 0b00011000;                                    // Enter config mode.
    WDTCSR =  0b00001000 | 0b000000;                         // clr WDIE (interrupt enable...7th from left; set WDE (reset enable...4th from left), and set delay interval
                                                             // reset system in 16 ms... unless wdt_disable() in loop() is reached first
  }
}


void PIR_Interrupt()  
{
  pir_sensor = 1;                                            // Interrupt PIR sensor
}


void BT_Interrupt()                                          // Interrupt Edukit switch
{
  Serial.println("\nPush button activated");
  SW_status = 1;
  detachInterrupt(1);
  acc_sensor = 0;
}


void ACC_Interrupt() 
{
  acc_sensor = 1;
}                                                            // Interrupt ACC sensor

//=====================================================================================================================================================================================
// Loop

void loop()
{
  Serial.println("\nHit the push button to get the counter error...");
  BT_status();
  if (SW_status)
  {
    error_status();
    SW_status = 0;
  }
  detachInterrupt(1);
  wdt_disable();
  sleep();
  attachInterrupt(1, ACC_Interrupt, LOW);   
  watchdogEnable();                                        // set up watchdog timer in interrupt-only mode
  sensor();
}

//=====================================================================================================================================================================================
// Additional Functions

void watchdogEnable()
{
  counter = 0;
  cli();                                                    // disable interrupts
  MCUSR = 0;                                                // reset status register flags
                                                            // Put timer in interrupt-only mode:                                        
  WDTCSR |= 0b00011000;                                     // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                                            // using bitwise OR assignment (leaves other bits unchanged).
  WDTCSR =  0b01000000 | 0b100001;                          // set WDIE (interrupt enable...7th from left, on left side of bar);  clr WDE (reset enable...4th from left)
                                                            // and set delay interval (right side of bar) to 8 seconds using bitwise OR operator.
  sei();                                                    // re-enable interrupts
}

    
void sleep()
{
  delay(1000);
  Serial.println("Sleeping...");
  mpu.enableSleep(true);                                    // Sleep MPU6050 to save battery
  delay(1000);
  ADCSRA &= ~(1 << 7);                                      // Disable ADC
  SMCR |= (1 << 2);                                         // Power down mode
  attachInterrupt(0, PIR_Interrupt, FALLING);               // Increase sensitivity. If, PNP sensors = RISING 
  SMCR |= 1;                                                // enable sleep
  digitalWrite(LED_BUILTIN, LOW);
  __asm__ __volatile__("sleep");
  delay(1000);
  Serial.println("\nThe MCU just woke up!");
  mpu.enableSleep(false);                                   // Wake up MPU6050
  delay(1000);
}


void sensor()
{
  if (pir_sensor)
  {
    pir_sensor = 0;
    detachInterrupt(0);   

    if (msg_cnt < MSG_LIMIT)
      sendMsgStd();
      
    else if (msg_cnt == MSG_LIMIT)
      {
        Serial.println("\n\nToo many sequence of Standard Messages sent!!!");
        sendAlarmMsg();
      }
  } 
  delay(IDLE_TIME);
  
  if (acc_sensor)
  {
    acc_sensor = 0;
    acc_flag   = 1;
    detachInterrupt(1);
    if (msg_cnt < MSG_LIMIT)
      sendMsgSpec();
      
    else if (msg_cnt == MSG_LIMIT)
      {
        Serial.println("\n\nToo many sequence of Standard Messages sent!!!");
        sendAlarmMsg();
      }
  }
  loop();
}


void sendMsgStd()
{
  voltage_shunt = ina219.getShuntVoltage_mV() / 1000.0;
  voltage_V = ina219.getBusVoltage_V() + voltage_shunt;
  current_mA = ina219.getCurrent_mA();

  if (error_flag)
  {
    msgID = 2;
    error_flag = 0;  
  }
  else
    msgID = 1;

  detachInterrupt(0);
  Serial.println("\n\nStandard Access");
  delay(3000);
  reset_();
  // | %s = string | %08lx = (l)long (x)hex in lowercase with 8 digits | %02x = hex in lowercase with 2 digits | %04x = hex in lowercase with 4 digits
  sprintf(buff, "AT+SEND=0:%04lx%04lx%02x;", voltage_V, current_mA, msgID);
  Serial.println("\nMessage to be sent: ");                               // (voltage 16 bits, current 16 bits, msgID 8 bits: ");
  Serial.println(buff);
  serial_HT.print (buff);     
  msgStatus();                                                          
}


void sendMsgSpec()
{
  voltage_shunt = ina219.getShuntVoltage_mV() / 1000.0;
  voltage_V = ina219.getBusVoltage_V() + voltage_shunt;
  current_mA = ina219.getCurrent_mA();

  if (error_flag)
  {
    msgID = 5;
    error_flag = 0;
  }
  else
    msgID = 4;
 
  detachInterrupt(1);
  Serial.println("\n\nSpecial Access");
  delay(3000);
  reset_();
  // | %s = string | %08lx = (l)long (x)hex in lowercase with 8 digits | %02x = hex in lowercase with 2 digits | %04x = hex in lowercase with 4 digits
  sprintf(buff, "AT+SEND=0:%04lx%04lx%02x;", voltage_V, current_mA, msgID);
  Serial.println("\nMessage to be sent: ");                               // (voltage 16 bits, current 16 bits, msgID 8 bits: ");
  Serial.println(buff);
  serial_HT.print (buff);
  msgStatus();   
}


void sendAlarmMsg()
{
  voltage_shunt = ina219.getShuntVoltage_mV() / 1000.0;
  voltage_V = ina219.getBusVoltage_V() + voltage_shunt;
  current_mA = ina219.getCurrent_mA();

  if (error_flag)
  {
    msgID = 8;
    error_flag = 0;
  }
  else
    msgID = 7;
    
  detachInterrupt(0);
  Serial.println("\nAlarm Message.");
  delay(3000);
  reset_();
  // | %s = string | %08lx = (l)long (x)hex in lowercase with 8 digits | %02x = hex in lowercase with 2 digits | %04x = hex in lowercase with 4 digits
  sprintf(buff, "AT+SEND=0:%04lx%04lx%02x;", voltage_V, current_mA, msgID);
  Serial.println("\nMessage to be sent: ");                               // (voltage 16 bits, current 16 bits, msgID 8 bits: ");
  Serial.println(buff);
  serial_HT.print (buff);
  msgStatus();  
}


void BT_status()
{
  attachInterrupt(1, BT_Interrupt, LOW);
  /*for (int k = 0; k <= 50; k++)
  {
    /*digitalWrite(LED_BUILTIN, HIGH);
    delay(40);
    digitalWrite(LED_BUILTIN, LOW);
    delay(40);
  } */
  delay(3000);
}


void error_status()
{
  Serial.print("The error counter is: ");
  Serial.println(error_check);
  Serial.println("");
  /*for (int j = 0; j <= 50; j++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }
  delay(500);
  for (int i = 0; i < error_check; i++)
  {
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  } */
  delay(1000);
}


void reset_()
{
  serial_HT.print("AT+RESET;");
  delay(1000);
  digitalWrite(RESET, HIGH);
  delay(1000);
  digitalWrite(RESET, LOW);
  delay(100);
  msgStatus();
  delay(1000);
  if (region)
  {
    Serial.println("\nConfiguring to the Region 1:");
    delay(1000);
    serial_HT.print("AT+CFGRCZ=1;");
  }
  else
  {
    Serial.println("\nConfiguring to the Region 2:");
    delay(1000);
    serial_HT.print("AT+CFGRCZ=2;");
  }
  msgStatus();
  delay(2000);  // 2000, para assegurar uma boa transnmissao evitando erros. 
}


void msgStatus()                                                                                    // Aguarda resposta, finaliza em 45 Segundos.
{
  uint16_t i;
  for(i=0; i<45000; i++)
  {
    if (serial_HT.available())
    {
      String answer = serial_HT.readStringUntil('\n');
      Serial.println("");
      Serial.println(answer);

      if((strcmp(answer.c_str(), "Error Send Frame: 161") == 0)||   //Error Send Frame: 161, Error Send Frame: 60
         (strcmp(answer.c_str(), "Error Send Frame: 60") == 0))
      {
          error_check++;
          error_flag = 1;
          Serial.println("\nSend message error. The MCU will reset...");
          errorMsg_handler();
      }       
                
      else if((strcmp(answer.c_str(), "AT_Cmd error: 0xA0") == 0)||
              (strcmp(answer.c_str(), "AT_Cmd error: 0xA1") == 0)||
              (strcmp(answer.c_str(), "AT_Cmd error: 0xA2") == 0)||
              (strcmp(answer.c_str(), "Open rcz error: 1") == 0))
             {
                error_check++;
                error_flag = 1;
                Serial.println("\nInternal communication error, the MCU will reset...");
                errorMsg_handler();
             }   

       else if(strcmp(answer.c_str(), "Error Send Frame: 0") == 0)
       {
          msg_cnt++;
          Serial.print("\nMessage sent successfully");
          Serial.print(" and the counter is: ");
          Serial.println(msg_cnt);
          pir_sensor = 0;
          error_flag = 0;
       }

       else
       {
          pir_sensor = 0;
          error_flag = 0;
       }
                                                
      if(strcmp(answer.c_str(), "AT_cmd_Waiting...") == 0)  // Se resposta "Waiting", i>45000 p/ sair do laço for
          i = 50000;
        
    }
    delay(10);
  }

}


void errorMsg_handler()
{
  delay(2000);
  if (acc_flag)
  {
    sendMsgSpec();
    loop();                                // para evitar travamentos
  }   
  
  else
  {
    if ((msg_cnt) >= (MSG_LIMIT + 1))
      {
        sendAlarmMsg();
        loop();                           // para evitar travamentos
      }
    
    else
      {
        sendMsgStd();
        loop();                           // para evitar travamentos
      }
  }
}
