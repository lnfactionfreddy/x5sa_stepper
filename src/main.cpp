
#include "Arduino.h"
#include "stm32f103xe.h"
//stepper
#include "TeensyStep.h"
//board
#include "../lib/pins_CHITU3D_common.h"

#include "HardwareSerial.h"
//#include "Regexp.h"

#include "STM32FreeRTOS.h"
#include <string.h> 
#include "queue.h"


//DMA UART
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_hal_uart.h"
#include "uart.h"
static SemaphoreHandle_t bin_sem;     // Waits for parameter to be read


Stepper M1(X_STEP_PIN, X_DIR_PIN), M2(Y_STEP_PIN,Y_DIR_PIN);   //STEP pin =  2, DIR pin = 3
StepControl controller;
Stepper* motorSet_A[] = {&M1, &M2};

//int initial_homing=0;
//bool ENDSTOP_1=false;
//bool ENDSTOP_2=false;

typedef struct home
{
  int initial_homing=0;
  bool ENDSTOP_1;
  bool ENDSTOP_2;
}home_var;

struct home par_home={0,false,false};


float_t STEPS_PER_MM=16*50.00;
float_t HOME_STEPS_PER_MM=16*50.00;

//SERIAL 
HardwareSerial debug(USART1);
HardwareSerial Printer(USART3);  

#define main_serial debug

struct GCODE_DATA
{
    char command[4];
    char letter[60];
    
}GCODE_DATA_READ;

QueueHandle_t GCODE_RING_BUFFER;


int size_char;
unsigned long count;

char *name = NULL;
int home=0;
//task
void Stepper_task( void *pvParameters );
void SerialRead_task( void *pvParameters );
void Command_task(void *pvParameters);
 
void ENDSTOP_M1(){
  debug.println("endstop_test");
  if(par_home.initial_homing!=0){
     par_home.ENDSTOP_1=true;
    debug.println("endstop");
    }
    
}
void ENDSTOP_M2(){
    if(par_home.initial_homing!=0){
     par_home.ENDSTOP_2=true;
    debug.println("endstop");
    }
}
void G1(char *letter){

}
void G2(char *letter){

}
int G28(void *pvParameters){
    xSemaphoreGive(bin_sem);
    par_home.initial_homing=1;
    while (par_home.initial_homing==1)
    {
    M1.setTargetRel((HOME_STEPS_PER_MM*10.00));
    controller.moveAsync(M1);

    xSemaphoreGive(bin_sem);
    if(par_home.ENDSTOP_1==true)
    {
        debug.println("home true");
        controller.stop();
        par_home.ENDSTOP_1=false;
        M1.setPosition(0);
        M1.setTargetAbs(-(HOME_STEPS_PER_MM*5.00));
        controller.move(M1);
        while (1)
        {
          M1.setTargetRel((HOME_STEPS_PER_MM*5.00));
          controller.moveAsync(M1);
          xSemaphoreGive(bin_sem);
         if(par_home.ENDSTOP_1==true){
          controller.stop();
          par_home.ENDSTOP_1=false;
          break;
         }
        }
        
        break;
    }
    
    }
    par_home.initial_homing=0;
    return 0;
}
void G(char *command, char *letter,void *pvParameters){
    char zahl[2];
    for( uint8_t i = 1 ; i<strlen(command) ; ++i){
        zahl[i-1]=command[i];   
    }
    int command_zahl = atoi(zahl);
    debug.println(command_zahl);
    switch (command_zahl)
    {
    case 1:
        G1(letter);
        break;
    case 2:
        G2(letter);
        break;
    case 28:
        debug.println("G28 Case function!!!");
        G28(pvParameters);
        break;
    default:
        break;
    }
}
void INIT_PINS(){
  pinMode(X_STOP_PIN, INPUT_PULLUP);
  pinMode(Y_STOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(X_STOP_PIN), ENDSTOP_M1, FALLING);
  attachInterrupt(digitalPinToInterrupt(Y_STOP_PIN), ENDSTOP_M2, FALLING);
  pinMode(X_ENABLE_PIN,OUTPUT);
  pinMode(Y_ENABLE_PIN,OUTPUT);
  digitalWrite(X_ENABLE_PIN,LOW);
  digitalWrite(Y_ENABLE_PIN,LOW);  
}
void INIT_MOTOR(){
M1
    .setMaxSpeed(32000)       // steps/s
    .setAcceleration(250000) // steps/s^2 
    .setStepPinPolarity(HIGH);
  
M2
    .setMaxSpeed(32000)       // steps/s
    .setAcceleration(250000) // steps/s^2 
    .setStepPinPolarity(HIGH);
}

void setup()
{
    
    INIT_PINS();
    INIT_MOTOR();
    //int eeprom_size = EEPROM.length();

    main_serial.begin(9600,SERIAL_8N1);
    while (!main_serial)
    {
     ; /* code */
    }
    GCODE_RING_BUFFER  = xQueueCreate(50  , sizeof( struct data* ) );
    bin_sem = xSemaphoreCreateBinary();
    /*
    xTaskCreate(
    Stepper_task
    ,  (const portCHAR *)"Stepper_task"   // A name just for humans
    ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  
    ,  NULL );
    */
  xTaskCreate(
    SerialRead_task
    ,  (const portCHAR *) "SerialRead_task"
    ,  1024  // Stack size
    ,  NULL
    ,  2  // Priority, with 7 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

      xTaskCreate(
    Command_task
    ,  (const portCHAR *) "Command_task"
    ,  1024  // Stack size
    ,  (void *)&par_home
    ,  2  // Priority, with 7 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

     vTaskStartScheduler();
    
}

void loop() 
{  
  ;  
}




void SerialRead_task(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  struct      GCODE_DATA   *GCODE_DATA_NOW;


  for(;;){
       if (debug.available() > 0) {
    String buffer= debug.readStringUntil('\n');
    // Length (with one extra character for the null terminator)
    // Length (with one extra character for the null terminator)
    //debug.println(str);
    int str_len = buffer.length() + 1; 
    // Prepare the character array (the buffer) 
    char char_array[str_len];
    // Copy it over 
    buffer.toCharArray(char_array, str_len);
    // Iterate over the source string (i.e. s) and cast the case changing.
    for (int a = 0; a < str_len; a++)
    {
        // Storing the change: Use the temp array while casting to uppercase.  
        char_array[a] = toupper(char_array[a]); 
    }
    debug.println(char_array);
    //MatchState ms (char_array);
    //count = ms.GlobalMatch ("(//ACTION:)", match_callback);
    //debug.println(strncmp("//ACTION:",char_array,9));
    if (0==strncmp("//ACTION:",char_array,9))
    {
      debug.println("Action TRUE");
        int i = 0;
        char *p = strtok (char_array, " ");
        char *array[3];

        while (p != NULL)
        {
          if(i==1){
            array[i++] = p;
            p = strtok (NULL,"\0");
            array[i++] = p;        
          }
          else{
        array[i++] = p;
        p = strtok (NULL, " ");
          }
        }
        if ( GCODE_RING_BUFFER != 0 ) 
            {
                debug.println(array[1]);
                debug.println(array[2]);
                strcpy(GCODE_DATA_READ.command,array[1]);
                strcpy(GCODE_DATA_READ.letter, array[2]);

                GCODE_DATA_NOW = &GCODE_DATA_READ;  
                if(xQueueSend ( GCODE_RING_BUFFER , (void *) &GCODE_DATA_NOW , 10 )!= pdTRUE){
                    debug.println("ERROR: Could not put item on delay queue.");
                }
            }
      
      /*
      switch (command[0])
      {
      case 'G':
        debug.println("G");
        break;
      default:
        break;
      }
      */
      }
    }
  }
}
/*
void Stepper_task(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for(;;){
      for(int i = 0; i < 5; i++)
  {
    M1.setTargetRel(spr); // 1/4 revolution
    controller.move(M1);  

    M1.setTargetRel(-spr);
    controller.move(M1);  
  }
  //delay(500);
  vTaskDelay((500L * configTICK_RATE_HZ) / 1000L);

  
  // move motor_1 to absolute position (10 revolutions from zero)
  // move motor_2 half a revolution forward  
  // both motors will arrive their target positions at the same time
  motorSet_A[0]->setTargetRel(10*spr);
  motorSet_A[1]->setTargetRel(-10*spr);
  controller.move(motorSet_A);

  }
}
*/
void Command_task(void *pvParameters)  // This is a task.
{
    struct      GCODE_DATA   *GCODE_DATA_Received;
    //pinMode(X_STOP_PIN, INPUT_PULLUP);
    //pinMode(Y_STOP_PIN, INPUT_PULLUP);
    xSemaphoreGive(bin_sem);

    for(;;){
      //home_var test=*((home_var*)pvParameters);
        if ( GCODE_RING_BUFFER )
                {
                    if ( uxQueueMessagesWaiting ( GCODE_RING_BUFFER ) )    
                    {
                        if(xQueueReceive ( GCODE_RING_BUFFER , &(GCODE_DATA_Received) , 0 )){
                            char command[]=" ";
                            char letter[]=" ";
                            strcpy(command,GCODE_DATA_Received->command);
                            strcpy(letter,GCODE_DATA_Received->letter);

                            switch (command[0]){
                            case 'G':
                            debug.println("G");
                            //debug.println(par_home.ENDSTOP_1);
                            G(command,letter, pvParameters);
                            break;

                            case '$':
                            debug.println("$");

                            break;

                            default:
                            break;
                            }

                        }

                }

    }
}

}
























/*
int str_len = str.length() + 1; 

    // Prepare the character array (the buffer) 
    char char_array[str_len];

    // Copy it over 
    str.toCharArray(char_array, str_len);

    // Iterate over the source string (i.e. s) and cast the case changing.
    for (int a = 0; a < str_len; a++)
    {
        // Storing the change: Use the temp array while casting to uppercase.  
        char_array[a] = toupper(char_array[a]); 
    }
    debug.println(char_array);
    //MatchState ms (char_array);
    //count = ms.GlobalMatch ("(//ACTION:)", match_callback);
    //if (strncmp("//ACTION:",char_array,9))
    //{
      
      char *token;
      token =strtok(char_array," ");
      token = strtok(char_array," ");
      strcpy(action,token);
      debug.println(action);
      token = strtok(char_array," ");
      strcpy(command,token);
      debug.println(command);
      token = strtok(char_array, "/n");
      strcpy(letter,token);
      debug.println(letter);
      
      switch (command[0])
      {
      case 'G':
        debug.println("G");
        break;
      default:
        break;
      }
    }*/
    