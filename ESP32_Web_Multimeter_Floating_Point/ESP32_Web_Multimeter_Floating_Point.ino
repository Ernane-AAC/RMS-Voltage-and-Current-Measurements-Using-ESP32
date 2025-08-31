// UNVERSIDADE FEDERAL DE UBERLÂNDIA - Faculdade de Engenharia Elétrica
// Subject: DSP-EPS - Prof. Ernane A. A. Coelho - http://lattes.cnpq.br/9183492978798433
// Code for RMS Voltage and Current Measurements 
//  * run on Esp32 Lab System (ESP32-WROOM-32 module)
//  * Sample rate ~=6kHz (6000.15 Hz)  ==> 100 samples per period (fundamental frequency at 60Hz)
//  * RMS calculation considering a moving average filter:
//    - summations ( Vk, Ik) are performed in floating point inside a specific task, since is not possible to run floating arithmetic inside the ISR (~=6kHz)
//    - the task is sinchronized with a timer ISR unsing the FreeRTOS function vTaskNotifyGiveFromISR()
//    - the task converts sample to Volt and Ampére and calculates the RMS values in floating point
//    - the RMS values are available for user by a LCD and a Webpage
// NOTE: In this proposal, the RMS values are made available at low frequency (1Hz), since they are intended 
//       for visualization on a LCD and web page. You can change the frequency according to the application, but keep 
//       in mind that that the task that consumes the generated data must accompany the one that produces it. 
// Additional Libraries used in this code:
//  -> LiquidCrystal_I2C: https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library/archive/refs/heads/master.zip
//  -> ESP32WebServer: https://github.com/Pedroalbuquerque/ESP32WebServer

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <WiFi.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32WebServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include <esp_task_wdt.h>
#include "driver/adc.h" //==> to use adc1_get_raw(channel)
//==> adc1_get_raw(channel)[~=42us] is faster than analogRead(channel)[~=52us]
//Definitions below are intrinsic of the adc library
//# define ADC1_CHANNEL_4 32  //GPIO32  -> Voltage channel
//# define ADC1_CHANNEL_5 33  //GPIO33  -> Current channel

# define Run_LED_Control 2  //connected to GPIO2

//definitions for timer 0
#define TIMER_GROUP           TIMER_GROUP_0  // Selects a Timer-Group 0 of 2 available groups(TIMER_GROUP_0=0/TIMER_GROUP_1=1)
#define TIMER_INDEX           TIMER_0        // Select a hardware timer from timer group (TIMER0=0/TIMER1=1)
#define TIMER_DIVIDER         67             // The divider’s range is from 2 to 65536. This code 67 ==> Timer_CLK = 80 MHz/67=1.19402985 MHz 
#define TIMER_TOP_COUNT       198            // 1.19402985 MHz/(TOP+1)= 1.19402985 MHz/199 => 6000.15 Hz
//There are different pairs of divider and Counter_Steps that can generate a carrier with a frequency close to 6 kHz. 
//Since divider and Counter_Steps are integers, it is not possible to generate a carrier with an exact frequency of 6 kHz. 
//The pair of integers divider and Counter_Steps, which implies the carrier frequency closest to 6kHz corresponds to:
// divider=67 and Counter_Steps=199 (or vice versa), resulting in the frequency of 6000.15 kHz. 
//Remember that since zero is a counter state, Counter_Steps corresponds to TOP+1.

int32_t sample_v, sample_i; //samples of voltage and current read from ADC1
float sum_v2;               //squared voltage summation 
float buffer_v2[100];       //squared voltage buffer 
float sum_i2;               //squared current summation 
float buffer_i2[100];       //squared current buffer 
int   buf_index;            //buffer pointer - used to access head and tail of FIFO
float average_v2,average_i2; // mean squared values

float gain_v,offset_v;     // voltage gain and offset obtained by multipoint calibration  
float gain_i,offset_i;     // current gain and offset obtained by multipoint calibration  
float Vrms=0, Irms=0;      // rms values
struct RMS_quantities
    {
      float Vrms;
      float Irms;
    };
struct RMS_quantities R;      // structure to receive RMS values from RMS_Calc Task
volatile int task_Counter;    // task counter                 

QueueHandle_t queue_RMS_Value;//Queue to transfer data from RMS_Calc Task to Loop
static TaskHandle_t task_handle = NULL;

// ----- wireless variables ----------------------------------  
const char* WL_SSID = "Esp32_Multimeter";
const char* WL_pass = "password";

String    webpage;         // string where it will be created the answer for webserver client
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
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Signal the task using task notification
    vTaskNotifyGiveFromISR(task_handle, &xHigherPriorityTaskWoken);

    // Notify the scheduler that it should yield control to a different task if needed
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

   timer_group_clr_intr_status_in_isr(TIMER_GROUP, TIMER_INDEX); //clear the interrupt flag for the timer 0 in Group 0 
   timer_group_enable_alarm_in_isr(TIMER_GROUP, TIMER_INDEX);    // enable alarm of the timer 0 in Group 0 for the next cycle
 }

//**************************************************************************************************************
//  SETUP
//**************************************************************************************************************
void setup() {
 bool esp32_AP; 
 pinMode(Run_LED_Control, OUTPUT);    // initialize digital pin Run_LED_Control as an output.
 digitalWrite(Run_LED_Control, LOW);  // turn the Run_LED off => external LED Drive connected in GPIO2   
 Serial.begin(115200);
 vTaskDelay(pdMS_TO_TICKS(500)); //wait 0.5s
 Serial.println("ESP32 Multimeter");
//-------config ADC  ------------------------
 adc1_config_width(ADC_WIDTH_BIT_12);
 adc1_config_channel_atten(ADC1_CHANNEL_4,ADC_ATTEN_DB_11); //the highest attenuation setting => input voltage range: 0 a 3.3V
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
  task_Counter=0;
  sum_v2=0;
  sum_i2=0; 
  // reset buffers for initial transient
  // irrelevant after the steady state of moving average filter is reached
  for(buf_index=0;buf_index<100;buf_index++)
     {
      buffer_v2[buf_index] = 0;
      buffer_i2[buf_index] = 0;   
     }
  buf_index=0;
 //-> coefficients to convert samples to Volt and Ampére obtained by multipoint calibration 
    gain_v = 0.279;
 offset_v = -523.69;
   gain_i = -0.0065728;
 offset_i = 12.338;

  queue_RMS_Value = xQueueCreate( 1, sizeof(struct RMS_quantities )); // create queue - 1 position of struct summation_samples
 
  //=> Create a task to calculate the RMS Values in core 1 - do not use the code 0 since the task will starves the IDLE0 (6kHz rate),
  //    which is the idle task for code 0 and it is monitored by the WDT, then this results in a WDT trigger.
  //=> Note that is important to create the task before ISR to prevent NULL handle notification 
  //  (vTaskNotifyGiveFromISR will assert fail because task_handle is still NULL)
  xTaskCreatePinnedToCore(RMS_Calc, "RMS_Calc", 8192, NULL, 1, &task_handle, 1);

  // --------- config timer and interrupt service routine ---------------------
  Config_Timer();
 
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
  server_init(); // webserver begin
  delay(5000);
  timer_enable_intr(TIMER_GROUP, TIMER_INDEX); //start interrupt operation, task is running and able to receive data
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
//  Task to calculate RMS Values
//**************************************************************************************************************
void RMS_Calc(void*z)
{
 struct RMS_quantities S;    //structure to send data to loop
 float Vinst, Iinst;         // instantaneous variables
 
 while (true) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));// Wait for ISR notification - timeout of 1ms > 167us

    sample_v = adc1_get_raw(ADC1_CHANNEL_4);  //GPIO32  -> voltage channel reading
    sample_i = adc1_get_raw(ADC1_CHANNEL_5);  //GPIO33  -> current channel reading 

    // --> remove first sample of FIFO (tail) from summation
    sum_v2 = sum_v2 - buffer_v2[buf_index]; 
    sum_i2 = sum_i2 - buffer_i2[buf_index]; 
 
    // --> insert new  squared sample to the respective FIFO (head)
    Vinst  = gain_v * (float)sample_v + offset_v; //calculates the sample in Volt
    Iinst  = gain_i * (float)sample_i + offset_i; //calculates the sample in Ampère
    buffer_v2[buf_index] = Vinst*Vinst;  //insert the squared value of voltage in FIFO
    buffer_i2[buf_index] = Iinst*Iinst;; //insert the squared value of current in FIFO

    // --> add new squared sample to summation
    sum_v2 = sum_v2 + buffer_v2[buf_index]; 
    sum_i2 = sum_i2 + buffer_i2[buf_index]; 
 
    // --> pointer control - circular buffer
    buf_index++;                            //advance pointer
    if(buf_index >= 100) buf_index = 0;     //reset pointer if it reaches the end (circular buffer)

    task_Counter++;
    if (task_Counter>=3000) //the purpose is to write values in a LCD and Webpage (low frequency: 2Hz)
       {                    //to plot rms curve save values in to a vector and plot offline
        task_Counter=0;  
        digitalWrite(Run_LED_Control, !digitalRead(Run_LED_Control)); //blink LED in 1Hz ==> RMS_Calc is running!
        average_v2 = sum_v2*0.01; //calculates the mean squared value
        average_i2 = sum_i2*0.01;
        S.Vrms=sqrt(average_v2);  //calculates the rms value
        S.Irms=sqrt(average_i2);
        xQueueSend(queue_RMS_Value, &S, NULL );// send RMS values to Loop  
       }
   }   
 }
 
//**************************************************************************************************************
// Timer Configuration
//**************************************************************************************************************
void Config_Timer(void) {
  timer_config_t config = {        // Data structure with timer’s configuration settings
    .alarm_en = TIMER_ALARM_EN,    // Timer alarm enable
    .counter_en = TIMER_PAUSE,     // Counter enable => 0 keep the timer paused, it will be started by timer_start().
    .intr_type = TIMER_INTR_LEVEL, // Interrupt mode
    .counter_dir = TIMER_COUNT_UP, // Counter direction
    .auto_reload = TIMER_AUTORELOAD_EN, // Timer auto-reload (Disable auto-reload: TIMER_AUTORELOAD_DIS=0/Enable auto-reload: TIMER_AUTORELOAD_EN=1)
    .divider = TIMER_DIVIDER,      // Counter clock divider. 
  };

  timer_init(TIMER_GROUP, TIMER_INDEX, &config);         // Initialize the hardware timer
  timer_set_counter_value(TIMER_GROUP, TIMER_INDEX, 0);  // Reset counter
  timer_set_alarm_value(TIMER_GROUP, TIMER_INDEX, TIMER_TOP_COUNT);  // Set timer top count value: Top=198, Divider=67 (80MHz/13333=6000.15)

  // ------ Register ISR handler ---------------------
    // esp_err_ttimer_isr_register(timer_group_t group_num, timer_idx_t timer_num, void (*fn)(void *), void *arg, int intr_alloc_flags, timer_isr_handle_t *handle)
    //              group_num: Timer group number
    //              timer_num: Timer index of timer group
    //                     fn: Interrupt handler function
    //                    arg: Parameter for handler function
    //       intr_alloc_flags: Flags used to allocate the interrupt. One or multiple (ORred) ESP_INTR_FLAG_* values 
    //                 handle: Pointer to return handle. If non-NULL, a handle for the interrupt will be returned here
  timer_isr_register(TIMER_GROUP, TIMER_INDEX, onTimer, nullptr, ESP_INTR_FLAG_IRAM, nullptr);

 // timer_enable_intr(TIMER_GROUP, TIMER_INDEX);
  timer_start(TIMER_GROUP, TIMER_INDEX);   // Start the timer
}


//**************************************************************************************************************
// SERVER init
//**************************************************************************************************************
 void server_init(void)
 { 
        Serial.println("Webserver starting..."); 
        server.on("/",      RMS_data); 
        server.begin();                         // Starting the webserver
        Serial.println("Webserver started..."); // Start the webserver 
 }

//**************************************************************************************************************
//   |==================================================================================================|
//   |=              WEB PAGE in HTML                                                                 =|
//   |==================================================================================================|
//**************************************************************************************************************
void RMS_data()
{ // Processes a client request 
  webpage = ""; // empties the string
  webpage  = "<!DOCTYPE html><head>";
  webpage += "<meta http-equiv=\"refresh\" content=\"5\">";
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
          
