// UNVERSIDADE FEDERAL DE UBERLÂNDIA - Faculdade de Engenharia Elétrica
// Subject: DSP-EPS - Prof. Ernane A. A. Coelho - http://lattes.cnpq.br/9183492978798433
// Code for RMS Voltage and Current Measurements 
//  * run on Esp32 Lab System (ESP32-WROOM-32 module)
//  * Sample rate ~=6kHz (6000.6 Hz)  ==> 100 samples per period (fundamental frequency at 60Hz)
//  * RMS calculation considering a moving average filter:
//    - summations of digital samples(Vk, Vk^2, Ik e Ik^2) are performed in integer arithmetic
//         inside the ISR (~=6kHz), since ESP32 does not support floating point operation inside the ISR
//    - every 1/2 second the ISR sends the summations for a specific task, which calculate the rms values
//    - the task converts sample summations to Volt and Ampére and calculates the RMS values in floating point
//    - the RMS values are presented in a 20x4 LCD connected in i2c bus and are also available using a Webserver
// NOTE: In this proposal, the RMS values are made available at low frequency (2Hz), since they are intended 
//       for visualization on a LCD and web page. You can change the frequency according to the application, but keep 
//       in mind that ESP32 runs over FreeRTOS considering a default tick of 100 Hz, then it is not possible
//       to transfer data between tasks in higher rates. 
// Additional Libraries used in this code:
//  -> LiquidCrystal_I2C: https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library/archive/refs/heads/master.zip
//  -> ESP32WebServer: https://github.com/Pedroalbuquerque/ESP32WebServer

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <ESP32WebServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/adc.h" //==> to use adc1_get_raw(channel)
//==> adc1_get_raw(channel)[~=36us] is faster than analogRead(channel)[~=52us]
//Definitions below are intrinsic of the adc library
//# define ADC1_CHANNEL_4 32  //GPIO32  -> Voltage channel
//# define ADC1_CHANNEL_5 33  //GPIO33  -> Current channel
# define Run_LED_Control 2  //connected to GPIO2

hw_timer_t * timer = NULL;
int32_t sample_v, sample_i; //samples of voltage and current read from ADC1
int32_t sum_vd;             //digital voltage summation 
int32_t sum_vd2;            //squared digital voltage summation 
int32_t buffer_vd[100];     //digital voltage buffer 
int32_t buffer_vd2[100];    //squared digital voltage buffer 
int32_t sum_id;             //digital current summation 
int32_t sum_id2;            //squared digital current summation 
int32_t buffer_id[100];     //digital current buffer 
int32_t buffer_id2[100];    //squared digital current buffer 
int   buf_index;            //buffer pointer - used to access head and tail of FIFO
struct summation_samples
    {
      int32_t sum_vd;
      int32_t sum_id;
      int32_t sum_vd2;
      int32_t sum_id2;
    };
struct summation_samples S; //struture to send samples from ISR

float gain_v,offset_v;     // voltage gain and offset obtained by multipoint calibration in fixed point 
float gain_i,offset_i;     // current gain and offset obtained by multipoint calibration in fixed point 
float Vrms, Irms;          // rms values
struct RMS_quantities
    {
      float Vrms;
      float Irms;
    };
struct RMS_quantities R;     //structure to receive RMS values from RMS_Calc Task
int intCounter;              // timer interrupt counter 
volatile int print_LCD;      // variable to control LCD print frequency                   

QueueHandle_t queue_samples; // Queue to transfer data from ISR to RMS_Calc Task
QueueHandle_t queue_RMS_Value;//Queue to transfer data from RMS_Calc Task to Loop

// ----- wireless variables ----------------------------------  
const char* WL_SSID = "Esp32_Multimeter";
const char* WL_pass = "password";

String    webpage;         //string where it will be created the answer for webserver client
ESP32WebServer server(80); // Start server on port 80 

// ----- LCD config ------------------------------------------
// Esp32 Lab System: Two LCD modules are connected to ESP32 i2c bus: SDA -> GPIO21 | SCL -> GPIO22
// Base address for chip PCF8574   = 0x27 (open jumpers)
// Base address for chip PCF8574A  = 0x3F (open jumpers)
LiquidCrystal_I2C lcd1(0x27,20,4);  //Left  side -> PCF8574 open jumpers           (HHH): Address = 0010 0(A2-A1-A0) -> 0x27
LiquidCrystal_I2C lcd2(0x26,20,4);  //Right side -> PCF8574 only A0 jumper shorted (HHL): Address = 0010 0(A2-A1-A0) -> 0x26

//**************************************************************************************************************
// Interrupt Service Routine - ISR
//**************************************************************************************************************
void IRAM_ATTR onTimer()  
{ 
 sample_v = adc1_get_raw(ADC1_CHANNEL_4);  //GPIO32  -> voltage channel reading
 sample_i = adc1_get_raw(ADC1_CHANNEL_5);  //GPIO33  -> current channel reading 

 //---------- update the summations of samples v, i, and squared sample v2 and i2 --------------
 // --> remove first sample of FIFO (tail) from summation
 sum_vd  = sum_vd  - buffer_vd[buf_index]; 
 sum_id  = sum_id  - buffer_id[buf_index];
 sum_vd2 = sum_vd2 - buffer_vd2[buf_index]; 
 sum_id2 = sum_id2 - buffer_id2[buf_index]; 
 
 // --> insert new sample and squared sample to the respective FIFO (head)
 buffer_vd[buf_index]  = sample_v; 
 buffer_id[buf_index]  = sample_i;
 buffer_vd2[buf_index] = sample_v*sample_v; //product fits in 32-bits
 buffer_id2[buf_index] = sample_i*sample_i; //product fits in 32-bits 

 // --> add new squared sample to summation
 sum_vd  = sum_vd  + buffer_vd[buf_index];  //
 sum_id  = sum_id  + buffer_id[buf_index];  //
 sum_vd2 = sum_vd2 + buffer_vd2[buf_index]; // summation fits in 32-bits
 sum_id2 = sum_id2 + buffer_id2[buf_index]; // summation fits in 32-bits
 
 // --> pointer control - circular buffer
 buf_index++;                            //advance pointer
 if(buf_index >= 100) buf_index = 0;     //reset pointer if it reaches the end (circular buffer)

 intCounter++;
 if (intCounter>=3000)   // update measurements for the consumer at 2Hz
   {                     // As FreeRTOS tick frequency is 100Hz, it is not possible to send/store calculated samples at 6kHz
     intCounter=0;       // As the idea here is to show the rms values to the user, the data will be updated at each second 
     S.sum_vd=sum_vd;    // do not use a faster update, since the task that consume data can not follow the producer
     S.sum_id=sum_id; 
     S.sum_vd2=sum_vd2;    
     S.sum_id2=sum_id2;
     xQueueSendFromISR(queue_samples, &S, NULL );// send summations to task
     print_LCD=1;        //used to sinc LCD print at each 1/2 second
                         //note that this implies a data tranfer between tasks using global variable, but in low frequency
   }  
}
//**************************************************************************************************************
//  SETUP
//**************************************************************************************************************
void setup() {
 bool esp32_AP;                       // boolean variable to check if Wifi is online
 pinMode(Run_LED_Control, OUTPUT);    // initialize digital pin Run_LED_Control as an output.
 digitalWrite(Run_LED_Control, LOW);  // turn the Run_LED off => external LED Drive connected in GPIO2   
 Serial.begin(115200);                // Serial interface begin
 Serial.println("ESP32 Multimeter");  // Send greeting message
 // ===> NOTE: The Serial interface here is used for debugging purpose, in normal operation it is recommended to 
 //      disconnect it and use a good quality 5V source to supply the ESP32, this will result in a 
 //      better voltage reference to ADC. Remember that you can not connect a 5V source to the ESP32 
 //      and keep the USB connection simultaneously.
 
//-------config ADC  ------------------------
  adcAttachPin(ADC1_CHANNEL_4);
  adcAttachPin(ADC1_CHANNEL_5);
  analogSetClockDiv(1);

 lcd1.begin();     //start LCD1
 lcd2.begin();     //start LCD2
 lcd1.backlight(); // turn on LCD backlight 
 lcd2.backlight(); // turn on LCD backlight 

 // send opening message to LCD's
 lcd1.setCursor(0,0);  // set cursor postion to row 1, column 1 
           //01234567890123456789
 lcd1.print("Digital Multimeter ");
 lcd1.setCursor(0,1);  // set cursor postion to row 2, column 1  
           //01234567890123456789
 lcd1.print("True RMS Value ");
 lcd1.setCursor(20,0); // set cursor postion to row 3, column 1 
           //01234567890123456789
 lcd1.print("Vrms: ");
 lcd1.setCursor(20,1); // set cursor postion to row 4, column 1 
           //01234567890123456789
 lcd1.print("Irms: ");
 lcd2.setCursor(0,0);  // set cursor postion to row 1, column 1 
           //01234567890123456789
 lcd2.print("Sample Rate: 6kS/s  ");

 //--------- Initializations ------------- 
  intCounter=0;
  print_LCD=0;
  sum_vd=0;
  sum_id=0; 
  sum_vd2=0;
  sum_id2=0; 
  Vrms=0;
  Irms=0;
  // reset buffers for initial transient, just a matter of formality
  // it doesn't matter after the steady state of moving average filter is achieved
  for(buf_index=0;buf_index<100;buf_index++)
     {
      buffer_vd[buf_index] = 0;
      buffer_id[buf_index] = 0; 
      buffer_vd2[buf_index] = 0;
      buffer_id2[buf_index] = 0;   
     }
  buf_index=0;
 //-> coefficients to convert samples to Volt and Ampére obtained by multipoint calibration 
 //-> Dados de calibração Jun/2023 - LV20_Rp=27k  LA55-> 4 espiras
   gain_v = 0.279;
 offset_v = -523.69;
   gain_i = -0.0065728;
 offset_i = 12.338;
 
 // Create queues to transfer data 
 queue_samples = xQueueCreate( 1, sizeof(struct summation_samples )); // create queue -> 1 position of struct summation_samples
 queue_RMS_Value = xQueueCreate( 1, sizeof(struct RMS_quantities )); // create queue -> 1 position of struct summation_samples
 
 // --------- config timer and interrupt service routine ---------------------
  timer = timerBegin(0, 4, true);    //For cpu_clk=80Mhz -> timer_clk=20MHz
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 3333, true);    //-> clk=20Mhz/3333 -> interrupt rate = 6000.6Hz  
  //the "timerAlarmEnable(timer)" will be executed after the RMS_Calc task starts
 
 //Create a task to calculate the RMS Values in core 0  
  xTaskCreatePinnedToCore(RMS_Calc, "RMS_Calc", 8192, NULL, 1, NULL, 0);

  Serial.println(" Creating the access point");
  esp32_AP = WiFi.softAP(WL_SSID, WL_pass, 2, 0, 4);//channel 2, broadcast SSID, 4 Max simultaneous connected clients
  if(esp32_AP)
     {
      Serial.println(" WiFi AP is online!");
      Serial.print("IP address: "); 
      Serial.println(WiFi.softAPIP());  //send IP address to serial 
      lcd2.setCursor(0,1);  // set cursor postion to row 2, column 1  
           //01234567890123456789
      lcd2.print("Wifi SSID:     ");
      lcd2.setCursor(20,0); // set cursor postion to row 3, column 1 
                //01234567890123456789
      lcd2.print("    Esp32_Multimeter");
      lcd2.setCursor(20,1); // set cursor postion to row 4, column 1 
                //01234567890123456789
      lcd2.print("Ip: ");   
      lcd2.setCursor(24,1); // set cursor postion to row 4, column 5 
               //01234567890123456789
               //IP: 255.255.255.255    
      lcd2.print(WiFi.softAPIP());       
     }
  else
     {
      Serial.println(" WiFi AP Failed!"); 
      Serial.println(" Restarting ESP32 in 10s!"); 
      lcd2.clear();
                //01234567890123456789
      lcd2.print("WiFi AP Failed!");
      lcd2.setCursor(0,1);  // set cursor postion to row 2, column 1  
           //01234567890123456789
      lcd2.print("Restarting     ");
      lcd2.setCursor(20,0); // set cursor postion to row 3, column 1 
                //01234567890123456789
      lcd2.print("      ESP32 in 10s!");   
      vTaskDelay(pdMS_TO_TICKS(10000)); //wait 10s
      ESP.restart(); 
     }
  // ---- webserver begin ---------------------------------
  Serial.println("Webserver starting..."); 
  server.on("/",      RMS_data); 
  server.begin();                         // Starting the webserver
  Serial.println("Webserver started..."); // Start the webserver 
  
  vTaskDelay(pdMS_TO_TICKS(5000)); //wait 5s;
  timerAlarmEnable(timer);  // start interrupt operation, task is running and able to receive data
}
//**************************************************************************************************************
//  LOOP
//**************************************************************************************************************
void loop()
 {
  char quantity1[5], quantity2[7];
  if(xQueueReceive(queue_RMS_Value, &R, portMAX_DELAY)== pdPASS )
      {
       Vrms=R.Vrms;
       Irms=R.Irms; 
      }
      
   if(print_LCD==1) //update RMS Values in the LCD at each 1/2 second
      {
       print_LCD=0; 
       dtostrf(Vrms, 5, 1,quantity1);
       sprintf(quantity2,"%5sV", quantity1); 
       lcd1.setCursor(26,0);  // set cursor postion to row 3, column 7  
               // 01234567890123456789
               //"Vrms: 220.1V ");
       lcd1.print(quantity2); //print Vrms on LCD1

       dtostrf(Irms, 5, 1,quantity1);
       sprintf(quantity2,"%5sA", quantity1);
       lcd1.setCursor(26,1);  //set cursor postion to row 4, column 7  
             // 01234567890123456789
             //"Irms:  10.1A ");
       lcd1.print(quantity2); //print Irms on LCD1
      }      
   server.handleClient();
   vTaskDelay(pdMS_TO_TICKS(200)); //wait 200ms
 }

//**************************************************************************************************************
//  Task to calculate RMS Values
//**************************************************************************************************************
void RMS_Calc(void*z)
 {
   char quantity1[5], quantity2[7];
   struct summation_samples R; //structure to receive data
   struct RMS_quantities S;    //structure to send data to webserver
   float Bv2,Bv1,Bv0;          // coefficients to calculate the average value of V2 in Volt
   float Bi2,Bi1,Bi0;          // coefficients to calculate the average value of I2 in Ampére
   float average_v2,average_i2; // mean squared values
   // calculate the ccoefficients
   Bv2=gain_v*gain_v*0.01;
   Bv1=2*gain_v*offset_v*0.01;
   Bv0=offset_v*offset_v;
   Bi2=gain_i*gain_i*0.01;
   Bi1=2*gain_i*offset_i*0.01;
   Bi0=offset_i*offset_i;
   Serial.println("RMS_Calc() is running in core: " + String(xPortGetCoreID()));
   for(;;)
      {
       if(xQueueReceive(queue_samples, &R, portMAX_DELAY)== pdPASS ) 
          {
           digitalWrite(Run_LED_Control, !digitalRead(Run_LED_Control)); //blink LED in 1Hz ==> RMS_Calc is running!  
           //--- rms calculations => here floating-point calculations are possible ------
           average_v2= Bv2*(float)R.sum_vd2 + Bv1*(float)R.sum_vd + Bv0; //calculate the average value of V2 in Volt
           S.Vrms=sqrt(average_v2);
           average_i2= Bi2*(float)R.sum_id2 + Bi1*(float)R.sum_id + Bi0; //calculate the average value of I2 in Ampére
           S.Irms=sqrt(average_i2);
           xQueueSend(queue_RMS_Value, &S, NULL );// send RMS values to Loop
          }              
     vTaskDelay(pdMS_TO_TICKS(100)); //wait 100ms
     }    
 }

//**************************************************************************************************************
// |=======================================================================================================|
// |              WEB PAGE in HTML                                                                         |
// |=======================================================================================================|
//**************************************************************************************************************
void RMS_data()
{ // Processes a client request 
  webpage = ""; // empties the string
  webpage  = "<!DOCTYPE html><head>";
  webpage += "<meta http-equiv=\"refresh\" content=\"5\">"; //refresh page at each 5s
  webpage += "<title>Esp32 Multimeter</title>";
  webpage += "<style> body { background-color: #fffff; font-family: Arial, Helvetica, Sans-Serif; Color: #000000;}"; 
  webpage += "h1 {color:darkblue;} h5 {color: darkcyan;} h2 {color: blue;}";
  webpage += "table, td, th { border: 1px solid darkcyan; font-size: 120%;}</style></head>";
  webpage += "<body><h1> ESP32 Multimeter </h1>";
  // *****  Data table begin ******
  webpage += "<div style=\"text-align: center; border: 2px solid darkcyan\">"; 
  webpage += "<h2>RMS Voltage and Current</h2>";
  webpage += " <table align=\"center\">";
  webpage += "<tr><th>Channel </th><th></th><th>Voltage (V)</th><th></th><th>Channel </th><th></th><th>Current (A)</th></tr>";
  webpage += "<tr><td>Ch4 </td><th></th><td>"+String(Vrms)+"</td><th></th><td>Ch5</td><th></th><td>"+String(Irms)+"</td></tr>";     
  webpage += "</table></div>";  
  // *****  Data table end ******
  webpage += "<footer><h5 align=\"right\">Powered by: NUPEP-FEELT-UFU</h5></footer></body></html>";

  server.send(200, "text/html", webpage);//
}
          
