/*
Credit to all whom deserve it for writing libraries. I could not have done 
this without the help of the community.

This sketch was writen to be easy to understand, not to help other people, 
but because my knowledge is limited and it is within my capabilities. 
Please take it and make it your own, improve it and let me know of any good 
ideas you may have.

archywigglestyn@gmail.com

one thing I would like to see fixed is the pid sample time. if it were to 
take an average of the last (given amount of time) say using some type of
first in first out (fifo) we could achieve a much more smooth operation in
the derivitive area due to the long hysteresis lag charactoristics of a smoker
(try turning the sample time down and the derivitive up. you'll see what I mean)
maybe I am just missing something, if so, let me know, that would be great.

Archy
7/28/15

Update 8/16/16
configured the analog pins as inputs instead of reading an anolog value for a 
digital purpose.


I/O List

Pins
0 down button wired N.O to 0vdc Input
1 up button wired N.O to 0vdc Input
2 lcd
3 lcd
4 lcd
5 lcd
6 thermoclk
7 thermocs
8 thermodo
9 lcd
10 backlight output
11 pulsed output for heating
12 lcd
13 pulsed output for smoke element. this will heat at a desired/fixed rate

A1 Auto Manual switch
A2 Screen Index Button


*/
//////////////////////
// Libraries 
/////////////////////
/////////////////////
// these include more text to get the job done and make it simple for dumb 
// people like me. you will need to download them and include them in the
// Arduino file on your computer


#include "Adafruit_MAX31855.h"

/*************************************************** 
  This is an example for the Adafruit Thermocouple Sensor w/MAX31855K

  Designed specifically to work with the Adafruit Thermocouple Sensor
  ----> https://www.adafruit.com/products/269

  These displays use SPI to communicate, 3 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <PID_v1.h>
#include <LiquidCrystal.h>

/*
 Library originally added 18 Apr 2008
 by David A. Mellis
 library modified 5 Jul 2009
 by Limor Fried (http://www.ladyada.net)
 example added 9 Jul 2009
 by Tom Igoe
 modified 22 Nov 2010
 by Tom Igoe

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/LiquidCrystalCursor

 */
#include <Servo.h>
#include <EEPROMex.h>

//////////////////////////
/////////////////////////
/////////////////////////




//Max31855 thermocouple amplifier settings////////////////////
int thermoDO = 8;
int thermoCS = 7;
int thermoCLK = 6;
Adafruit_MAX31855 thermocouple(thermoCLK, thermoCS, thermoDO);

// pid settings ///////////////////////////////////////

double Setpoint, Input, Output;
double consKp=1.5, consKi=.02, consKd= 10;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

//////////////////////////////////////////////////////////
// LCD

LiquidCrystal lcd(12, 9, 2, 3, 4, 5); // follow the directions in on adafruit
// for setting up the lcd. they are very helpful



//eeprom 
void updateAndReadDouble(); //to aid in making it easy to store doubles 

//////////////////////////////////////////////////////////
int upbuttonstate; // shows the current state of the button
int downbuttonstate;  // shows the current state of the button
int lastupstate = 0;  // used to save the last state of the button
int lastdownstate = 0;  // used to save the last state of the button
long upbuttontime = 0;  // used with millis to determine time held down
long downbuttontime = 0; // used with millis to determine time held down
int buttondelay; // how long button is held before fast indexing.
int backlight; // back light state high or low
long backlightcount; // used with millis to determine time backlight has spent on
int backlighttime; // time the light stays on
int automanswitch = 0; // used with the analog read to make a switch digital. manual auto pid control
double thermoread; // a tempoorary storage byte for temp logic
long temppausecount; //used with millis to determine time the temp pauses
int temppauseontime = 0; // time before the temppause turns on to update the temp
int temppauseofftime; //time before the temppause turns off and timer resets after updating temp 
int temppause;// a binary state to determine whether or not to update the temp
int maxsetpoint; //maximum allowible setpoint
int minsetpoint; // minimum allowable setpoint
double outputperc; // used as an interface between pid and non pid use to simplify the sketch
int screenindex; // used to read the state of the button to index through screens also analog
int screen; // represents the number of screen that is desired
int indexup =0; // a digital state used to index up in the current screen
int indexdown =0; // same thing just down
int lastindexstate =0; // used for debouncing the index button
int pidsample; // used to make the pid sample time adjustable
int outpulse; // pulsed output for heating element
long prepulse; // counter for pulse interval
int relayon; // used to switch on the output. helps indecate high or low
int relayoff; // the same just off
int smokeon; //same as relayon/relaypff only instead it is for the smoke element
int smokeoff;
long smoketimer; // used for timing the smoke element intervals
int smokeperc; // the percentage the smoke element will run


////////////////////////////////////////////////////////////////////////////////////



void setup() {

  
  
///////////////////////////////////////////////////////
//PID

  myPID.SetOutputLimits(0,255); // not actually needed since it's default is 0,255 but it's here if you want it
  myPID.SetSampleTime(pidsample); // just as it says
  
////////////////////////////////////////////////////

pinMode(0, INPUT_PULLUP); //sets pullup resistors to keep the inputs from floating
pinMode(1, INPUT_PULLUP); //sets pullup resistors to keep the inputs from floating
pinMode(10, OUTPUT);
pinMode(11, OUTPUT);
pinMode(13, OUTPUT);
pinMode(A1, INPUT);
pinMode(A2, INPUT);


  temppause = 1; // set to 1 to start everything off
  digitalWrite(10, HIGH); // turns the back light on
  backlightcount = millis(); // sets the count so the backlight turns on.
  

  
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // clears the lcd
  lcd.clear();
  // Print a message to the LCD.
  lcd.print("Smoke-A-Matic!"); // YOU CAN TYPE ANYTHING YOU WANT HERE 16 charactors
  // Turn on the display:
  lcd.display();
  
  // wait for MAX chip and display to stabilize and show "your message"
  delay(1000);
  // clears the lcd
  lcd.clear();
  
  //myservo.attach(13);
  
  ///////////////////////////////////////////////////////////////////////////////////////
  // adjustable values all in one place for convenience
 // servofulopen = 45; // sets the open position of servo in degrees
 // servofulclosed = 128; // sets the closed position of servo in degrees
  Setpoint = 225; // sets the initial setpoint temp off the start
  backlighttime = 30000; // time the light stays on
  temppauseontime = 1000; // time before the temppause turns on to update the temp
  temppauseofftime = 1100; //time before the temppause turns off and timer resets after updating temp
  maxsetpoint = 450; //maximum allowible setpoint
  minsetpoint = 60; // minimum allowable setpoint
  Output = 0; //had a problem i could not solve, with out the program making a cycle the Output would always start at 45.6
  pidsample = 5000; // used to make the pid sample time adjustable
  buttondelay = 1000;// how long button is held before fast indexing. you can choose how long hear
 relayon = LOW;// we want our output to be low when calling for power
 relayoff = HIGH;// and high when not.
 smokeon = LOW;
 smokeoff = HIGH;
}


void loop() {
  ////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////
   ///////////////////////////////////////////////////////////
  

      automanswitch = digitalRead(A1);
      screenindex = digitalRead(A2);
   
  ////////////////////////////////////////////////////////////
  //Temp, update delay, used to keep the temp 
  //from being all scatter brained.
  
  thermoread = (thermocouple.readFarenheit()); // reads the thermocouple value into thermoread

  if (thermoread < 1000 && temppause == 1){   //<1000 is used to keep an infinint error out
    Input = thermoread;                       // temppause allows it to update
  }
 
  
  // update delay section updates temp based on timed setpoints 
  // this makes the temp more readable as it can change back and forth quite rapidly
  if ((millis() - temppausecount) > temppauseontime) {
    temppause = 1;}
    
    else {temppause = 0;}                 // this section is a typical timer, my daughter needs
                                          // play time with dad so i'm not going to explain it, 
  if ((millis() - temppausecount) > temppauseofftime) {  //but there is lots of good info online
     temppausecount = millis();}
 
   ///////////////////////////////////////////////////////////////////////////////  
  //PID stuff
  if (automanswitch == 1) {myPID.SetMode(AUTOMATIC); // sets PID to auto 
  }
  
  if (automanswitch == 0) {myPID.SetMode(MANUAL);} // sets PID to manual
 
  
  
  myPID.SetTunings(consKp, consKi, consKd); // inputs our setting for PID
  
  myPID.Compute(); // gives a commute command

  myPID.SetSampleTime(pidsample); // inputs sample time
   
 
  //////////////////////////////////////////////////////////////////
  //up button count logic debounce
  // indexes the setpoint and turns the backlight on
  // indexes at a fast rate after button is held for a given time
  upbuttonstate = digitalRead(1); // reads the state of the button
  
  if (backlight == HIGH){                      //if backlight is on and 
    if (upbuttonstate != lastupstate) {        // the button position changed and
      if(upbuttonstate == LOW && downbuttonstate == HIGH) {// the button is pressed and the other is not
          indexup = 1;}}}                     // allows the screen to index. gets set back to 0 at end of sketch
      
  if (upbuttonstate == HIGH) {                  //resets the counter saying the button is not held
    upbuttontime = millis();}                    //counter for fast index
    
  if ((millis() - upbuttontime) > buttondelay) { //if millis exceeds button delay
    indexup = 1; backlightcount = millis();}      // fast indexes and also turns on backlight
  
  if (upbuttonstate != lastupstate) {           //detects button state change
   backlightcount = millis();}                // turns on the backlight using the button   
  
  
 
  
  ////////////////////////////////////////////////////////////
  //down button logic debounce
  // indexes the setpoint and turns the backlight on
  // indexes at a fast rate after button is held for a given time
 
  
  downbuttonstate = digitalRead(0);
  if (backlight == HIGH) {
    if (downbuttonstate != lastdownstate) {
      if(downbuttonstate == LOW && upbuttonstate == HIGH) {
        indexdown = 1;}}}                     
                                        // this is all the same as the above, just for the down button
  if (downbuttonstate == HIGH) {
    downbuttontime = millis();}                  
    
  if ((millis() - downbuttontime) > buttondelay) {
    indexdown = 1; backlightcount = millis();}    
      
  if (downbuttonstate != lastdownstate) {
     backlightcount = millis();} 
  
  
  
  ///////////////////////////////////////////////////////////
  // screen index logic
  // also follow the same logic as above just for the index button
  
  if (backlight == HIGH){
    if (screenindex != lastindexstate) {
      if(screenindex == 0) { // remember this is an anolog input so it's a little different
          screen++, lcd.clear();}}} // clears and indexes the screen if the backlight is on
          
  if (screenindex != lastindexstate) {
     backlightcount = millis();} // turns on the backlight using the button
     
  if (screen > 7){screen = 0;} // limits the screen number and starts it back to 0

  /////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////
  //lcd logic printing

// screen 0 to start us off. asks if you want to load from epprom 
  if(screen==0){

 lcd.setCursor(0, 0);
    lcd.print("Hold up & down");
    lcd.setCursor(0, 1);
    lcd.print("to load Settings");
    
      if (upbuttonstate == LOW && downbuttonstate == LOW){// if both buttons are on
        //we use the states here instead of the debounced indexes to get past the one or the other
        //shuttle. this allows me to keep logic local and other setting from indexing after the screen changes
        consKp = EEPROM.readDouble(0);// read the eeprom. from that address given
        consKi = EEPROM.readDouble(5);// doubles take 4 bytes, I thought
        consKd = EEPROM.readDouble(10);// but I needed 5 for it to work
        Setpoint=EEPROM.readDouble(15);
            screen++, lcd.clear();// this actually worked out really well. it does not index until 
                                  // the eeprom is done reading. that way you have some feedback that
                                  // the task is complete
 
 }}

  ////////////////////////////////////////////////////
  // screen 1
  if (screen == 1){
    lcd.setCursor(0, 0);
    lcd.print("Temp");
    lcd.print(Input);
    lcd.print("out");
    lcd.print(outputperc);
    lcd.setCursor(0, 1);
    lcd.print("Setpoint ");
    lcd.print(Setpoint);
    
    if (automanswitch == 1){// this section allows you to change the setpoint if
      if (indexdown == 1){Setpoint--;}  // the switch is set to auto
      if (indexup == 1) {Setpoint++;}}

    if (automanswitch == 0){          // this section allows you to adjust the output if and when
      if (indexdown == 1){Output-=2;} // the switch is set to manual
      if (indexup == 1) {Output+=2;}  // indexes by 2 to make it faster you can change this
      if (Output<0){Output=0;}        // limits the ranges
      if (Output>255){Output=255;}    //    ''    ''    ''
      }}
  
  if (Setpoint < minsetpoint) {  // the min allowable setpoint
    Setpoint = minsetpoint;}   
  if (Setpoint > maxsetpoint) {   // the max allowable setpoint
    Setpoint = maxsetpoint;}

    // this may not be the best way of limiting things, but it's what I came up with and stuck with it
 
  
  //screen 2 //////////////////for changing proportional 
  if (screen == 2){                         // this is all pretty simple daughter still waiting
    lcd.setCursor(0, 0);
    lcd.print("Proportional");
    lcd.setCursor(0, 1);
    lcd.print(consKp);
      if (indexdown == 1){consKp = (consKp-.01);}
      if (indexup == 1) {consKp = (consKp+.01);}
}
  
  // screen 3 //////////////// for changing integral
  if (screen == 3){
    lcd.setCursor(0, 0);
    lcd.print("Integral");
    lcd.setCursor(0, 1);
    lcd.print(consKi);
      if (indexdown == 1){consKi = (consKi-.01);}
      if (indexup == 1) {consKi = (consKi+.01);}
}
  
  // screen 4 /////////////// for changing derivative
  if (screen == 4){
    lcd.setCursor(0, 0);
    lcd.print("derivative");
    lcd.setCursor(0, 1);
    lcd.print(consKd);
      if (indexdown == 1){consKd = (consKd-.01);}
      if (indexup == 1) {consKd = (consKd+.01);}
}

   // screen 5 ////////////// for changing pid sample time////////
  if (screen == 5){
    lcd.setCursor(0, 0);
    lcd.print("PID Sample Time");
    lcd.setCursor(0, 1);
    lcd.print(pidsample);
      if (indexdown == 1){pidsample-= 100;}
      if (indexup == 1) {pidsample+= 100;}
}
    
   // screen 6 used for eeprom saving /////////////
  if (screen == 6){
    lcd.setCursor(0, 0);
    lcd.print("Hold up & down");
    lcd.setCursor(0, 1);
    lcd.print("to save Settings");
      if (upbuttonstate == LOW && downbuttonstate == LOW){
        EEPROM.updateDouble(0,consKp);
        EEPROM.updateDouble(5,consKi);
        EEPROM.updateDouble(10,consKd);
        EEPROM.updateDouble(15,Setpoint); 
          screen++, lcd.clear();// just like reading after task is done screen indexes
          

      }}
   
 // screen 7 used for setting the smoke element /////////////
  if (screen == 7){
    lcd.setCursor(0, 0);              // after you get these next two set change the screen number limit
    lcd.print("Smoke element");      // as you should not need it any more
    lcd.setCursor(0, 1);
    lcd.print(smokeperc);
    lcd.print("%");
      if (indexdown == 1){smokeperc-=5;}
      if (indexup == 1) {smokeperc+=5;}
      if (smokeperc<0){smokeperc=0;}        // limits the ranges
      if (smokeperc>100){smokeperc=100;}    //    ''    ''    ''
      
}

 //////////////////////////////////////////////////////////
  // output math
  outputperc=(Output/2.55); // makes a percentage to read on the lcd

if (outputperc>=100){digitalWrite(11,relayon);}// a strange problem here. this is to keep
  else{       // the relays from chattering when the timer reset
    if ((millis() - prepulse) < (outputperc*100)){ // a timer that pulses "on" based
        digitalWrite(11 ,relayon);}                         // on the output
    else {digitalWrite(11, relayoff);}}

    
  if ((millis() - prepulse) >= 10000){  // here, said timers off time is the remaining
    prepulse = millis();}             // amount withing the 255 bit range.
  



//smoke element//////////////////////////////////////////////////////
if (smokeperc>=100){digitalWrite(13,smokeon);} //Same as above
  else{
        if ((millis() - smoketimer) < smokeperc*100){ // a timer that pulses "on" based
            digitalWrite(13 , smokeon);}                         // on the smoke setting
        else {digitalWrite(13 , smokeoff);}}

    
  if ((millis() - smoketimer) >= 10000){  // here, said timers off time is the remaining
    smoketimer = millis();}     


    
 
  //////////////////////////////////////////////////////////////////
  // Backlight logic 
  
    
  if ((millis() - backlightcount) < backlighttime) { // basic counter that gets triggered thoughout the sketch
    backlight = HIGH;}
    
    else {backlight = LOW;}
  
   
    digitalWrite(10, backlight); // writes the state to pin number ten, operates backlight
    
   
  
///////////////////////////////////////////////////////////////
//end of line resets
    lastdownstate = downbuttonstate;  // sometimes the end of a program is not the best place for things
    lastupstate = upbuttonstate;      // like this always remember the order of operation in a loop. it goes down
    lastindexstate = screenindex;
    indexup = 0;
    indexdown = 0;
   delay(10);
}
