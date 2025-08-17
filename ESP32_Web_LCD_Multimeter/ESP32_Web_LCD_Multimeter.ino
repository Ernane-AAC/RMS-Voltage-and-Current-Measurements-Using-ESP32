// UNVERSIDADE FEDERAL DE UBERLÂNDIA - Faculdade de Engenharia Elétrica
// Subject: DSP-EPS - Prof. Ernane A. A. Coelho - http://lattes.cnpq.br/9183492978798433
// Code for RMS Voltage and Current Measurements 
//  * run on Esp32 Lab System (ESP32-WROOM-32 module)
//  * Sample rate ~=6kHz (6000.15 Hz)  ==> 100 samples per period (fundamental frequency at 60Hz)
//  * RMS calculation considering a moving average filter:
//    - summations of digital samples(Vk, Vk^2, Ik e Ik^2) are performed in integer arithmetic
//         inside the ISR (~=6kHz), since ESP32 does not support floating point operation inside the ISR
//    - every 1 second the ISR sends the summations for a specific task, which calculate the rms values
//    - the task converts sample summations to Volt and Ampére and calculates the RMS values in floating point inside the loop task
//    - the RMS values are presented in a 20x4 LCD connected in i2c bus and are also available using a Webserver
// NOTE: In this proposal, the RMS values are made available at low frequency (1Hz), since they are intended 
//       for visualization on a LCD and web page. You can change the frequency according to the application, but keep 
//       in mind that that the task that consumes the generated data must accompany the one that produces it.
// Additional Libraries used in this code:
//  -> LiquidCrystal_I2C: https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library/archive/refs/heads/master.zip
//  -> ESP32WebServer:    https://github.com/Pedroalbuquerque/ESP32WebServer

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
#include "esp_intr_alloc.h"
#include "driver/timer.h"
#include "driver/adc.h" //==> to use adc1_get_raw(channel)
                        //==> adc1_get_raw(channel)[~=36us] is faster than analogRead(channel)[~=52us]
# define Run_LED_Control 2  //connected to GPIO2
#define TIMER_GROUP           TIMER_GROUP_0  
#define TIMER_INDEX           TIMER_0        
#define TIMER_DIVIDER         67   // The divider’s range is from 2 to 65536. This code 67 ==> Timer_CLK = 80 MHz/67=1.19402985 MHz          
#define TIMER_TOP_COUNT       198  // 1.19402985 MHz/(TOP+1)= 1.19402985 MHz/199 => 6000.15 Hz
//There are different pairs of divider and Counter_Steps that can generate a carrier with a frequency close to 6 kHz. 
//Since divider and Counter_Steps are integers, it is not possible to generate a carrier with an exact frequency of 6 kHz. 
//The pair of integers divider and Counter_Steps, which implies the carrier frequency closest to 6kHz corresponds to:
// divider=67 and Counter_Steps=199 (or vice versa), resulting in the frequency of 6000.15 kHz. 
//Remember that since zero is a counter state, Counter_Steps corresponds to TOP+1.

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

float gain_v,offset_v;     // voltage gain and offset obtained by multipoint calibration 
float gain_i,offset_i;     // current gain and offset obtained by multipoint calibration 
float Vrms, Irms;          // rms values
float Bv2,Bv1,Bv0;          // coefficients to calculate the average value of V2 in Volt
float Bi2,Bi1,Bi0;          // coefficients to calculate the average value of I2 in Ampére
float average_v2,average_i2; // mean squared values


volatile int intCounter;     // timer interrupt counter 

QueueHandle_t queue_samples; // Queue to transfer data from ISR to RMS_Calc Task

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
void IRAM_ATTR onTimer(void *args)  
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
 if (intCounter>=6000)   // update measurements for the consumer at 1Hz
   {                     // Although the xQueueSendFromISR and xQueueReceive functions are not speed-limited by the FreeRTOS tick rate, 
     intCounter=0;       //     the goal here is to display RMS data on an LCD and webpage, so a high rate is not appropriate. 
     S.sum_vd=sum_vd;    // For faster applications, such as storing curves as vectors in memory, it's important to verify that
     S.sum_id=sum_id;    //     the task consuming the data can keep up with the one producing it.
     S.sum_vd2=sum_vd2;    
     S.sum_id2=sum_id2;
     xQueueSendFromISR(queue_samples, &S, NULL );// send summations to task                       
   }  
  timer_group_clr_intr_status_in_isr(TIMER_GROUP, TIMER_INDEX); //clear the interrupt flag for the timer 0 in Group 0 
  timer_group_enable_alarm_in_isr(TIMER_GROUP, TIMER_INDEX);    // enable alarm of the timer 0 in Group 0 for the next cycle
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
 adc1_config_width(ADC_WIDTH_BIT_12);
 adc1_config_channel_atten(ADC1_CHANNEL_4,ADC_ATTEN_DB_11);
 adc1_config_channel_atten(ADC1_CHANNEL_5,ADC_ATTEN_DB_11);

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
 //-> Calibration data Jun/2023 - LV20_Rp=27k  LA55-> 4-turn
   gain_v = 0.279;
 offset_v = -523.69;
   gain_i = -0.0065728;
 offset_i = 12.338;

   //determine the coefficients for RMS value calculation
   Bv2=gain_v*gain_v*0.01;
   Bv1=2*gain_v*offset_v*0.01;
   Bv0=offset_v*offset_v;
   Bi2=gain_i*gain_i*0.01;
   Bi1=2*gain_i*offset_i*0.01;
   Bi0=offset_i*offset_i;
 
 // Create queues to transfer data 
  queue_samples = xQueueCreate( 1, sizeof(struct summation_samples )); // create queue -> 1 position of struct summation_samples
  
  Serial.println(" Creating the access point");
  esp32_AP = WiFi.softAP(WL_SSID, WL_pass);//default: channel 1, 4 Max simultaneous connected clients
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
   
  // --------- config timer and interrupt service routine ---------------------
  Config_Timer();
  Serial.println("Timer was configured...");
  Serial.println("ISR was installed...");
  vTaskDelay(pdMS_TO_TICKS(5000)); //wait 5s
  timer_enable_intr(TIMER_GROUP, TIMER_INDEX);
}
//**************************************************************************************************************
//  LOOP
//**************************************************************************************************************
void loop()
 {
  char quantity1[5], quantity2[7];
  struct summation_samples R; //struture to receive samples from ISR

  if(xQueueReceive(queue_samples, &R, pdMS_TO_TICKS(100)) == pdPASS ) //wait for new squared average value (100ms)
     {
      digitalWrite(Run_LED_Control, !digitalRead(Run_LED_Control)); //blink LED in 0.5Hz ==> RMS_Calc is running!  
      //--- rms calculations => here floating-point calculations are possible ------
      average_v2= Bv2*(float)R.sum_vd2 + Bv1*(float)R.sum_vd + Bv0; //calculate the average value of V2 in Volt
      Vrms=sqrt(average_v2);
      average_i2= Bi2*(float)R.sum_id2 + Bi1*(float)R.sum_id + Bi0; //calculate the average value of I2 in Ampére
      Irms=sqrt(average_i2);
 
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
 }

//**************************************************************************************************************
// Timer configuration function
// The functions below allows the specification of divider and counter top. This can allow the user to get the
// desired frequency for the sample rate. Same Arduino functions has the frequency as the parameter, which is easier
// to use, but this may result in a frequency which is not the best approximation to the desired frequency
//**************************************************************************************************************
void Config_Timer(void) {
    timer_config_t config = {       
        .alarm_en = TIMER_ALARM_EN,    // Enable the alarm
        .counter_en = TIMER_PAUSE,     // Start the timer in paused state
        .intr_type = TIMER_INTR_LEVEL, // Set interrupt type to edge-triggered
        .counter_dir = TIMER_COUNT_UP, // Count up mode
        .auto_reload = TIMER_AUTORELOAD_EN, // Enable auto-reload
        .divider = TIMER_DIVIDER,     // Timer divider for frequency control
    };

    // Initialize the timer
    timer_init(TIMER_GROUP, TIMER_INDEX, &config);        
    
    // Set the initial counter value
    timer_set_counter_value(TIMER_GROUP, TIMER_INDEX, 0);  
    
    // Set the alarm value for the timer (defines the timer frequency)
    timer_set_alarm_value(TIMER_GROUP, TIMER_INDEX, TIMER_TOP_COUNT);  

    // Register the ISR (Interrupt Service Routine)
    timer_isr_register(TIMER_GROUP, TIMER_INDEX, onTimer, nullptr, ESP_INTR_FLAG_IRAM, nullptr);

    // Enable the timer interrupt
    //timer_enable_intr(TIMER_GROUP, TIMER_INDEX); //it will be enabled at the end of setup

    // Start the timer
    timer_start(TIMER_GROUP, TIMER_INDEX);   
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
          
