/*
   25/02/2023 V2.5 by Eng. Rafamuratt
   https://www.youtube.com/channel/UC1H5V-FIN7DCLTPmZ0Cm4zw

   This firmware works on Arduino, allowing data messages sending over the Sigfox network to a web dashboard called TagoIO. 
   The dashboard can be accesed using this link (registering and signing in with your own credentials is needed first): https://610efe6f94a33700127f37f9.tago.run
   The messages has some Id's in order to identity them between a normal message, special message, resend them (in case of error)and, to protect your Sigfox credits 
   (in case of issues), an alarm message can be sent too (it stop the local services until manual reset of device).

   Hardware used:
   # Arduino Nano (where this firmware has to run);
   # Edukit Redfox (Sigfox SoC HT32SX radio frequency transmitter/ receiver); https://curtocircuito.com.br/kit-livro-iot-sigfox-com-edukit-redfox.html
   # Some ordinary PIR sensor - detect movement around;
   # Acceleration sensor MPU6050 - detect 3D vibration (x,y and z axes);
   # Power check sensor INA219 - reads the instant voltage and current required from power supply
   # Two batteries Samsung INR18650-35E (3.7V 3.5Ah each) connected in series.

   <<<<<  Note: With the current set up and code to save power, the batteries can last up to 4 days  >>>>>
   
   Overall code explanation:
   It makes Arduino deep sleep to save battery. It only wake up after the PIR sensor detection (Interrupt 0 at D2). 
   As long the MSG_LIMIT is not over, it will send a "Standard Message" (ID = 1, or in case of error, ID = 2) 
   or "Special Message" (ID = 4, or in case of error, ID = 5) via Sigfox, to update a TagoIO dashboard.
   If the MSG_LIMIT its over, an AlarmMsg will be sent with (ID = 7, or in case of error, ID = 8) and the Message services are stopped.
   
   <<<<< The message content is: battery status ("voltage" 16 bits, "current" 16 bits + "msgID" 8 bits) >>>>>
 
   Note: Due to the HT32SX has physical UART (TX/ RX) connected to Arduino Nano D2 and D3,
   to use the Interruptions 0 and 1 (also located at D2 and D3)an hardware change is required.
   When it's done, there´s no conflict between interruptions and, the UART communication pins with the chip HT32SX
   After this change D11 and D12 (for example) can be used as TX/RX - SoftwareSerial.
   The USART communication uses an AT protocol with detailed explanation here: https://github.com/htmicron/ht32sx/tree/master_2/AT_Commands
   
   Also the connector (not the port) D5 is used to activate the interrupt 1 (D3) as the Edukit has a push button (SW) physically connected to it.
   This way, if SW is pressed in the right moment (as soon the system wake up) the error counter status is showed at Serial Monitor for debug purposes. 
   If enabled, the same error counter can also be seen at the Arduino onboard led wich blinks with the following behaviour:
   1) the led blinks very fast (50 times in order to bring attention)
   2) after a short pause (0,5s) the led will blink each second (in the same quantity of current error status)

   Note 2: The acceleration sensor is also connected to the interrupt 1 (D3) to track some vibration and if this is the case, the "Special Message" is sent after the "Standard Message".

   In case of communication issues between Arduino and the EduKit,  the functions msgStatus() and errorMsg_handler() are implemented to avoid any dead lock and to send those 
   messages anyways, but with corresponding error ID for tracking.
   But if in case the firmware still hangs for more than 320s (COUNTMAX = 40 * 8 seconds), the interrupt-based Watchdog Timer will overflow (beacuse it won't be reseted at next loop). 
   This way, the reset-only Watchdog Timer of 16ms clear the flags and restart the Arduino.

   The Watchdog delay interval patterns are (flag WDTCSR):
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

#define MSG_LIMIT     30                   // limit to send mensages
#define IDLE_TIME     300000               // 5 min pause, to avoid sending too many msg
#define COUNTMAX      40                   // interrupt-based Watchdog Timer expires after about 320 secs 5,33 min (if 8 sec interval is selected).
#define DET_THRESHOLD 7                    // sensitivity to detect the vibration    
#define DET_DURATION  3                    // vibration time (dafault = 20)                                                                               
#define RESET         4                    // D4 Reset of HT32SX (Edukit) 



//=====================================================================================================================================================================================
// Objects Definition

SoftwareSerial serial_HT(12, 11);          // object serial_HT to connect to the HT32SX:  serial_HT(12, 11) = D12, D11 USART
Adafruit_INA219 ina219(0x40);              // object ina219 (addr. 0x40): check the battery voltage and current
Adafruit_MPU6050 mpu;                      // object mpu: detect the vibration



//=====================================================================================================================================================================================
// Global Variables

volatile int counter;                      // count number of times ISR is called.
int  error_check       = 0;                // error counter
int  msg_cnt           = 0;                // count of standard msg sent
int  msgID             = 0;
int  pir_sensor        = 0;
int  acc_sensor        = 0;
char PIR               = 2;                // D2
char ACC               = 3;                // D3 parallel connection with D5 (Edukit push button)
char error_flag        = 0;                // flag to try send the msg again, according with current state of msgID
char acc_flag          = 0;                // flag to send acc error msg (msgID = 5)
char SW_status         = 0;                // test the push button SW
char region            = 1;                // region definition, 1 = Europa
char buff[36];                             // buffer to convert the string.
float voltage_V        = 0.0;
float current_mA       = 0.0;
float voltage_shunt;



//=====================================================================================================================================================================================
// Functions Prototypes

void PIR_Interrupt();                      // wake up the system, check for some move around (interrupt 0, Arduino D2)
void ACC_Interrupt();                      // detect vibration (interrupt 1, Arduino D3)
void BT_Interrupt();                       // debug purpose, check the error counter (interrupt 1, Arduino D3)
void watchdogEnable();                     // set the watchdog timer mode (interrupt-only) 
void sleep();                              // deep sleep to save battery
void sensor();                             // check the sensors (PIR and acceleration)
void sendMsgStd();                         // send the "Standard Message", ID1 and in case of error, send the ID2
void sendMsgSpec();                        // send the "Special Message", ID4 and in case of error, send the ID5    
void sendAlarmMsg();                       // send the "Alarm Message", ID7 and in case of error, send the ID8
void msgStatus();                          // check the status after a message is sent, sending output to the errorMsg_handler()
void errorMsg_handler();                   // in case of error, try to send the message again
void error_status();                       // debug to retrieve the number of errors while sending a message to the Sigfox network
void reset_();                             // reset the Edukit Redfox ( SoC HT32SX), set the Sigfox region



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
  
  Serial.begin(9600);                                        // 9600 bps --> Serial Monitor (PC)
  serial_HT.begin(9600);                                     // 9600 bps --> chip HT32SX (Edukit)
  ina219.begin();
  mpu.begin();
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(DET_THRESHOLD);
  mpu.setMotionDetectionDuration(DET_DURATION);              
  mpu.setMotionInterrupt(true);                              // true = enable the hardware based interruption (threshold and duration). False = disable it.
  mpu.setInterruptPinLatch(false);                           // true = keep it latched (will turn off when reinitialized). False = reset after 50us
  mpu.setInterruptPinPolarity(true);                         // true = low when interrupt is active. False = high
  delay(1000);
  reset_();
 
}



//=====================================================================================================================================================================================
// Interruptions
// Watchdog interruption, force to go to sleep after some time defined in COUNTMAX 

ISR(WDT_vect)                                                // watchdog timer interrupt service routine
{
  counter+=1;
  if (counter < COUNTMAX)
    wdt_reset();                                             // start timer again (still in interrupt-only mode)
    
  else                                                       // then change timer to reset-only mode with short (16 ms) fuse
  {
    Serial.println("###################################");   // print anything, its a debug just to track the reset-based Watchdog Timer using the Serial Monitor
    delay(5000);
    MCUSR = 0;                                               // reset flags
                                                             // put timer in reset-only mode:
    WDTCSR |= 0b00011000;                                    // enter config mode.
    WDTCSR =  0b00001000 | 0b000000;                         // clr WDIE (interrupt enable...7th from left; set WDE (reset enable...4th from left), and set delay interval
                                                             // reset system in 16 ms... unless wdt_disable() in loop() is reached first
  }
}



//=====================================================================================================================================================================================
// wake up the system, check for some move around (interrupt 0, Arduino D2)

void PIR_Interrupt()  
  { pir_sensor = 1;}                                 



//=====================================================================================================================================================================================
// detect vibration (interrupt 1, Arduino D3)

void ACC_Interrupt()                                         
  {acc_sensor = 1;}   


  
//=====================================================================================================================================================================================
// debug purpose, check the error counter (interrupt 1, Arduino D3)

void BT_Interrupt()                                          
{
  Serial.println("\nPush button activated");
  SW_status = 1;
  detachInterrupt(1);
  acc_sensor = 0;
}


                                                   
//=====================================================================================================================================================================================
// Loop

void loop()
{
  Serial.println("\nHit the push button to get the counter error...");
  attachInterrupt(1, BT_Interrupt, LOW);
  delay(3000);
  if (SW_status)
  {
    error_status();
    SW_status = 0;
  }
  detachInterrupt(1);
  wdt_disable();
  sleep();
  attachInterrupt(1, ACC_Interrupt, LOW);   
  watchdogEnable();                                        
  sensor();
}



//=====================================================================================================================================================================================
// Additional Functions

// watchdogEnable(): set the watchdog timer mode (interrupt-only)

void watchdogEnable()
{
  counter = 0;
  cli();                                                    // disable interrupts
  MCUSR = 0;                                                // reset status register flags
                                                            // put timer in interrupt-only mode:                                        
  WDTCSR |= 0b00011000;                                     // set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                                            // using bitwise OR assignment (leaves other bits unchanged).
  WDTCSR =  0b01000000 | 0b100001;                          // set WDIE (interrupt enable...7th from left, on left side of bar);  clr WDE (reset enable...4th from left)
                                                            // and set delay interval (right side of bar) to 8 seconds using bitwise OR operator.
  sei();                                                    // re-enable interrupts
}



//=====================================================================================================================================================================================
// deep sleep to save battery

void sleep()
{
  delay(1000);
  Serial.println("Sleeping...");
  mpu.enableSleep(true);                                    // sleep MPU6050 to save battery
  delay(1000);
  ADCSRA &= ~(1 << 7);                                      // disable ADC
  SMCR |= (1 << 2);                                         // power down mode
  attachInterrupt(0, PIR_Interrupt, FALLING);               // increase sensitivity, NPN logic. If uses PNP sensors, set to RISING 
  SMCR |= 1;                                                // enable sleep
  digitalWrite(LED_BUILTIN, LOW);
  __asm__ __volatile__("sleep");
  delay(1000);
  Serial.println("\nThe MCU just woke up!");
  mpu.enableSleep(false);                                   // wake up MPU6050
  delay(1000);
}



//=====================================================================================================================================================================================
// check the sensors (PIR and acceleration)

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



//=====================================================================================================================================================================================
// send the "Standard Message", ID1 and in case of error, send the ID2

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
  sprintf(buff, "AT+SEND=0:%04lx%04lx%02x;", voltage_V, current_mA, msgID);   // %04lx = (l)long (x)hex with 4 digits | %02x = hex with 2 digits
  Serial.println("\nMessage to be sent: ");                                   // voltage 16 bits, current 16 bits, msgID 8 bits
  Serial.println(buff);
  serial_HT.print (buff);     
  msgStatus();                                                          
}



//=====================================================================================================================================================================================
// send the "Special Message", ID4 and in case of error, send the ID5

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
  sprintf(buff, "AT+SEND=0:%04lx%04lx%02x;", voltage_V, current_mA, msgID);
  Serial.println("\nMessage to be sent: ");                               
  Serial.println(buff);
  serial_HT.print (buff);
  msgStatus();   
}



//=====================================================================================================================================================================================
 // send the "Alarm Message", ID7 and in case of error, send the ID8

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
  sprintf(buff, "AT+SEND=0:%04lx%04lx%02x;", voltage_V, current_mA, msgID);
  Serial.println("\nMessage to be sent: ");                              
  Serial.println(buff);
  serial_HT.print (buff);
  msgStatus();  
}



//=====================================================================================================================================================================================
// check the status after a message is sent, sending output to the errorMsg_handler()

void msgStatus()                                                 // wait ca. 45 seconds for the answer.
{
  uint16_t i;
  for(i=0; i<45000; i++)
  {
    if (serial_HT.available())
    {
      String answer = serial_HT.readStringUntil('\n');
      Serial.println("");
      Serial.println(answer);

      if((strcmp(answer.c_str(), "Error Send Frame: 161") == 0)||   
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
                                                
      if(strcmp(answer.c_str(), "AT_cmd_Waiting...") == 0)     // interrupt the loop for in case this string is received
          i = 50000;
        
    }
    delay(10);
  }

}



//=====================================================================================================================================================================================
// in case of error, try to send the message again
 
void errorMsg_handler()
{
  delay(2000);
  if (acc_flag)
  {
    sendMsgSpec();
    loop();                                // to avoid deadlock
  }   
  
  else
  {
    if ((msg_cnt) >= (MSG_LIMIT + 1))
      {
        sendAlarmMsg();
        loop();                           // to avoid deadlock
      }
    
    else
      {
        sendMsgStd();
        loop();                           // to avoid deadlock
      }
  }
}



//=====================================================================================================================================================================================
// debug to retrieve the number of errors while sending a message to the Sigfox network

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



//=====================================================================================================================================================================================
// reset the Edukit Redfox ( SoC HT32SX), set the Sigfox regionn

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
  delay(2000);                                                   // to garantee a transmittion without errors. 
}
