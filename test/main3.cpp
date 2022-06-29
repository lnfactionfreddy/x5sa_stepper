#include <Arduino.h>
#include "TeensyStep.h"
#include "../lib/pins_CHITU3D_common.h"
#include "EEPROM.h"
#include "HardwareSerial.h"
#include "Regexp.h"
#include <stdio.h>  //<-- You need this to use printf.
#include <string.h>  //<-- You need this to use string and strlen() function.
#include <ctype.h>  //<-- You need this to use toupper() function.

//const int addressEEPROM_min = 0;              // Specify the address restrictions you want to use.
//const int addressEEPROM_max = 4095;



Stepper M1(X_STEP_PIN, X_DIR_PIN), M2(Y_STEP_PIN,Y_DIR_PIN);   //STEP pin =  2, DIR pin = 3

StepControl controller;
Stepper* motorSet_A[] = {&M1, &M2};
long initial_homing=-1;
bool ENDSTOP_1=false;
bool ENDSTOP_2=false;

float_t STEPS_PER_MM=16*5.00;

// put your main code here, to run repeatedly:
constexpr int spr = 16*200;  // 3200 steps per revolution


//SERIAL 
HardwareSerial debug(USART1);
HardwareSerial Printer(USART3);  
#define main_serial debug

int size_char;
unsigned long count;

//Regex 
/*

int useRegex(char *textToCheck) {
    regex_t compiledRegex;
    int reti;
    int actualReturnValue = -1;
    char messageBuffer[100];

    // Compile regular expression 
    reti = regcomp(&compiledRegex, "//ACTION:", REG_EXTENDED | REG_ICASE);
    if (reti) {
        fprintf(stderr, "Could not compile regex\n");
        return -2;
    }

    // Execute compiled regular expression 
    reti = regexec(&compiledRegex, textToCheck, 0, NULL, 0);
    if (!reti) {
        puts("Match");
        actualReturnValue = 0;
    } else if (reti == REG_NOMATCH) {
        puts("No match");
        actualReturnValue = 1;
    } else {
        regerror(reti, &compiledRegex, messageBuffer, sizeof(messageBuffer));
        fprintf(stderr, "Regex match failed: %s\n", messageBuffer);
        actualReturnValue = -3;
    }

  // Free memory allocated to the pattern buffer by regcomp() 
    regfree(&compiledRegex);
    return actualReturnValue;
}
*/

// called for each match
void match_callback  (const char * match,          // matching string (not null-terminated)
                      const unsigned int length,   // length of matching string
                      const MatchState & ms)      // MatchState in use (to get captures)
{
char cap [10];   // must be large enough to hold captures
  
  main_serial.print ("Matched: ");
  main_serial.write ((byte *) match, length);
  main_serial.println ();
  
  for (byte i = 0; i < ms.level; i++)
    {
    main_serial.print ("Capture "); 
    main_serial.print (i, DEC);
    main_serial.print (" = ");
    ms.GetCapture (cap, i);
    main_serial.println (cap); 
    }  // end of for each capture

}  // end of match_callback 

void ENDSTOP_M1(){
ENDSTOP_1 = true;


}

void ENDSTOP_M2(){
ENDSTOP_2 = true;

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
    .setAcceleration(300000) // steps/s^2 
    .setStepPinPolarity(HIGH);
  
M2
    .setMaxSpeed(32000)       // steps/s
    .setAcceleration(300000) // steps/s^2 
    .setStepPinPolarity(HIGH);
}

void setup()
{
    //INIT_PINS();
    //INIT_MOTOR();
    //int eeprom_size = EEPROM.length();
    main_serial.begin(9600,SERIAL_8N1);
    while (!main_serial)
    {
      ;
    }
    delay(300);
    //main_serial.println(eeprom_size);
  //unsigned long count;

  // what we are searching (the target)
  //char buf [100] = "//action:G28 //action:G28";

  // match state object
  //MatchState ms (buf);

  // original buffer
  //main_serial.println (buf);

  // search for three letters followed by a space (two captures)
  //count = ms.GlobalMatch ("(//action:)", match_callback);

  
 

  
}


void loop() 
{  
   if (main_serial.available() > 0) {
    String str = main_serial.readStringUntil('\n');
    // Length (with one extra character for the null terminator)
    // Length (with one extra character for the null terminator)
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
  	MatchState ms (char_array);
  
    main_serial.println(char_array);
    count = ms.GlobalMatch ("(//ACTION:)", match_callback);
    // show results
    main_serial.print ("Found ");
    main_serial.print (count);            // 8 in this case
    main_serial.println (" matches.");

  }
}


/*
  // lets shake    
  for(int i = 0; i < 5; i++)
  {
    M1.setTargetRel(spr); // 1/4 revolution
    controller.move(M1);  

    M1.setTargetRel(-spr);
    controller.move(M1);  
  }
  delay(500);
  
  // move motor_1 to absolute position (10 revolutions from zero)
  // move motor_2 half a revolution forward  
  // both motors will arrive their target positions at the same time
  motorSet_A[0]->setTargetAbs(10*spr);
  motorSet_A[1]->setTargetRel(-10*spr);
  controller.move(motorSet_A);

  // move all motors back to their start positions
  motorSet_A[0]->setTargetAbs(0);
  motorSet_A[1]->setTargetAbs(0);
  controller.move(motorSet_A);
 
  delay(1000);
  */
  

   /*
 while (!ENDSTOP_1) {  // Make the Stepper move CCW until the switch is activated   
    M1.setTargetAbs(initial_homing);  // Set the position to move to
    initial_homing--;  // Decrease by 1 for next move if needed
    controller.move(M1);  // Start moving the stepper

}
ENDSTOP_1=false;
//M1.setPosition(0);
M1.setTargetAbs(70);
controller.move(M1);

 while (!ENDSTOP_1) {  // Make the Stepper move CCW until the switch is activated   
    M1.setTargetAbs(initial_homing);  // Set the position to move to
    initial_homing--;  // Decrease by 1 for next move if needed
    controller.move(M1);  // Start moving the stepper

}
M1.setPosition(0);
*/


/*
  M1.setTargetRel(spr*10);
  M2.setTargetRel(spr*10);
  controller.move(M1,M2);
  delay(2000);
  */