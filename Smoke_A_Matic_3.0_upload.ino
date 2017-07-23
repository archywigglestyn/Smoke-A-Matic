/*
Credit to all whom deserve it for writing libraries. I could not have done 
this without the help of the community.

This sketch was writen to be easy to understand, not to help other people, 
but because my knowledge is limited and it is within my capabilities. 
Please take it and make it your own, improve it and let me know of any good 
ideas you may have.

archywigglestyn@gmail.com

One thing I would like to see fixed is the pid sample time. If it were to 
take an average of the last (given amount of time) say using some type of
first in first out (fifo) we could achieve a much more smooth operation in
the derivitive area due to the long hysteresis lag characteristics of a smoker
(try turning the sample time down and the derivitive up. you'll see what I mean)
maybe I am just missing something, if so, let me know, that would be great.

Archy
7/28/15

Setting servo positions. enter 90 into the open and closed settings below in the
(// adjustable values all in one place for convenience) area. 90 is the number of
degrees that is the center of a 180 degree servo. This will hopefully keep from
breaking anything.
Next upload the sketch
Next attach your servo assuring that the servo is indeed somewhere in the center of travel
Next turn you automan switch to manual then scroll through the menu and set your 
servo positions. Remember these numbers and enter them below in the same place we
just changed to 90
Now upload and your open and closed positions should match your setup perfectly
enjoy


3.0 Update 7/11/17 this controller now uses thermistors instead of thermocouples
thanks to Adafruits thermistor tutorial, I converted it into a function that allows
us to read multiple thermistors. I used replacment thermistors for Maverick ET732/ET733
meat thermometers, so the settings should be close. Given the way the code is writen
you need to have matched thermistors. I figured this would not be a problem since they 
are commercially available for many units.

This sketch contains three versions hobby servo, fan, and heating element. chose which is 
best for you and uncomment the sections you want. Note you can only use one version at a time

The heating element version uses two elements one for main heat and one for smoke. This works like a 
charm. I like to place the smoke element in something enclosed that can take the heat. With just a small
opening or a couple of holes to let smoke out. This also insulates the chamber from the smoker heat source
allowing you to do some pretty cold smoking.

The fan version uses PWM to control and output for a fan. There is an adjustable value that controls the percentage
of output that will be PWMed compared to pulsed on and off. Since some fan will not opperate at very low pulse widths.

The servo version uses a hobby servo as a damper controller. pretty straight forward.


I/O List

Pins
0 down button wired N.O 0vdc to Input
1 up button wired N.O 0vdc to Input
2 lcd
3 lcd
4 lcd
5 lcd
6 Auto Manual Switch  N.O 0vdc to Input
7 Screen Index Button  N.O 0vdc to Input
8 alarm light / beeper
9 lcd
10 lcd backlight output
11 pulsed output for heating electrically OR pwm fan output
12 lcd
13 pulsed output for smoke element. OR servo.

A0 internal pit temp termistor     -0vdc-----thermistor-----input|pin----10k resistor----+5vdc-
A1 Meat probe 1 also thermistors, wired the same way
A2 Meat probe 2
A3 Meat Probe 3


*/
//////////////////////
// Libraries 
/////////////////////
/////////////////////
// these include more text to get the job done and make it simple for dumb 
// people like me. you will need to download them and include them in the
// Arduino file on your computer

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

#include <EEPROMex.h>



////////////////////////////////////////////////////////////////////
// servo
/////////////////////////////////////////////////////////////////

#include <Servo.h>
Servo myservo;


/////////////////////////
// thermistor settings/////////////////////////////////////////

// resistance at 25 degrees C
#define THERMISTORNOMINAL 1000000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 24
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 10
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 4080
// the value of the 'other' resistor
#define SERIESRESISTOR 100000
float fahrenheit;
int samples[NUMSAMPLES];


int THERMISTORPIN;



// pid settings ///////////////////////////////////////
// I set this to work with my smoker. yours will probably be different.
// Learn how to tune PID it is very helpful.
double Setpoint, Input, Output;
double consKp=1.5, consKi=.02, consKd= 10;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

//////////////////////////////////////////////////////////
// LCD

LiquidCrystal lcd(12, 9, 2, 3, 4, 5); // follow the directions in on adafruit
// for setting up the lcd. they are very helpful



//eeprom 
void updateAndReadDouble(); //to aid in making it easy to store doubles on eeprom

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
int automanswitch = 0; // manual/auto output control switch
int maxsetpoint; //maximum allowable setpoint
int minsetpoint; // minimum allowable setpoint
double outputperc; // used as an interface between pid and non pid use to simplify the sketch. converts to percentage
int screenindex; // used to read the state of the button to index through screens also analog
int screen; // represents the screen number that is desired
int indexup =0; // a digital state used to index up in the currently screened value
int indexdown =0; // same thing just down
int lastindexstate =0; // used for debouncing the index button
int pidsample; // used to make the pid sample time adjustable
int outpulse; // pulsed output for heating element or fan
long prepulse; // counter for pulse interval
int prange;// percentage range of output to be pulsed used for fan output. at a low percentage
//output the fan will pulse on and off since some fans will not run at a low pwm speed
int meat1; //meat probes
int meat2;
int meat3;
int probestate;// a number that indecates which meat probes are on or off
int alarmtemp; // the alarm temp. temp at which the alarm turns on
int alarm;  //  the alarm state on/off
long alarmblink = 0; // the alarm blink timer
int alarmon;// used for send a high or low output when on
int alarmoff;// used to send a high or low output when off
int alarmlast = 0; // used to save the last state of the alarm. In order to switch the screen when the alarm turns on

int relayon; // used to switch on the output. send high or low of your choice
int relayoff; // the same just off
int smokeon; //same as relayon/relayoff only instead it is for the smoke element
int smokeoff;
long smoketimer; // used for timing the smoke element intervals
int smokeperc; // the percentage the smoke element will run
long temptimer; // used to take the temp at an interval holding a millis value

double servofulopen; // sets the open position of servo in degrees
double servofulclosed; // sets the closed position of servo in degrees
double servomath; //used to calculate the servo variable





void setup() {
  //////////////////////if needed for servo//////////
 // myservo.attach(13);
  ///////////////////////////////////////////////
  
///////////////////////////////////////////////////////
//PID

  myPID.SetOutputLimits(0,255); // not actually needed since it's default is 0,255 but it's here if you want it
  myPID.SetSampleTime(pidsample); // takes the sample time from our pidsample
  
////////////////////////////////////////////////////

pinMode(0, INPUT_PULLUP); //sets pullup resistors to keep the inputs from floating
pinMode(1, INPUT_PULLUP); //sets pullup resistors to keep the inputs from floating
pinMode(6, INPUT_PULLUP); //sets pullup resistors to keep the inputs from floating
pinMode(7, INPUT_PULLUP); //sets pullup resistors to keep the inputs from floating

pinMode(10, OUTPUT);
pinMode(11, OUTPUT);
pinMode(13, OUTPUT);
pinMode(8, OUTPUT);



 
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
  
  
  Output = 0; //had a problem i could not solve, with out the program making a cycle the Output would always start at 45.6
  ///////////////////////////////////////////////////////////////////////////////////////
  // adjustable values all in one place for convenience
   screen = 5;// preferred starting screen
  Setpoint = 225; // sets the initial setpoint temp of the chamber
  backlighttime = 30000; // time the light stays on in millis
   maxsetpoint = 450; //maximum allowible setpoint
  minsetpoint = 110; // minimum allowable setpoint
  pidsample = 5000; // used to make the pid sample time adjustable in millis
  buttondelay = 1000;// how long button is held before fast indexing.
  alarmtemp = 175;// the starting temp for the meat probe alarm to be set at. tells you when it's done cooking
  alarmon = HIGH;// used for send a HIGH or LOW output when on
  alarmoff = LOW;// used to send a HIGH or LOW output when off
 
  
 relayon = HIGH;// If used...used to switch on the output. send high or low of your choice
 relayoff = LOW;//If used... and here we put the opposite.
 smokeon = HIGH;//If used... same as above
 smokeoff = LOW;//If used... same as above

 servofulopen = 90; // If used...sets the open position of servo in degrees
 servofulclosed = 90; // If used...sets the closed position of servo in degrees

 
 prange = 30; // If used...percentage range of output to be pulsed when using the fan output
}


void loop() {
  ////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////
   ///////////////////////////////////////////////////////////
  

      automanswitch = digitalRead(6);
      screenindex = digitalRead(7);
   
  ////////////////////////////////////////////////////////////
  //Temp stuff as well as some random, but needed things


if((millis()-temptimer) > 3000){  // I like to think of this as a one shot timer. it's very simple
                                  //after it is triggered it immediately resets also reading the temperatures one time
     Input=tempRead(A0);  // reading the temp of the chamber/pit probe
     
     if (probestate>=1){    // the following is looking to see if we have the meat probes turned on using the probesate
        meat1=tempRead(A1);   // as our indicator. if so we read the temp if not, we set them to 0
      }
      else{meat1=0;}

      
     if (probestate>=2){
        meat2=tempRead(A2);
     }
      else{meat2=0;}

      
     if (probestate>=3){
        meat3=tempRead(A3);
      }
      else{meat3=0;}
     temptimer=millis(); //reset the timer
     lcd.clear();       //update the screen with the new temps
  }

Setpoint=constrain(Setpoint,minsetpoint,maxsetpoint);   //constrains the Setpoint to the min and max settings


      switch(probestate){               //this was a little hard to do without retyping things many times
        case 1:{if(meat1>alarmtemp){    // in this section the alarm watches the probes that are on through 
           alarm=1;}                    // probestate and a switch case. if the probe goes of the set alarmtemp
        }                               // the it turns on
        break;

        case 2:{if(meat1>alarmtemp&&meat2>alarmtemp){
              alarm=1;}
        }
        break;

        case 3:{if(meat1>alarmtemp&&meat2>alarmtemp&&meat3>alarmtemp){
              alarm=1;}
        }
        break;
      }


//// here we blink an output when the alarm is on
  if(alarm==1){
      if ((millis() - alarmblink) < 80){ // this value is the on time. you can change this to what you want
            digitalWrite(8, alarmon);}
      else {digitalWrite(8, alarmoff);}}

    
  if ((millis() - alarmblink) > 2000){  // this is the off time. you can change this too
    alarmblink = millis();} 

  if (alarm==1){    //this section switches the screen to say the food is done
    if(alarm != alarmlast){
      screen=7;alarmlast=1;
    }
  }

   if (alarm==0){  // here we reset alarm last if the alarm turns off
    alarmlast=0;
   }
   ///////////////////////////////////////////////////////////////////////////////  
  //PID stuff
  if (automanswitch == 0) {myPID.SetMode(AUTOMATIC);} // sets PID to auto 
  
  
  if (automanswitch == 1) {myPID.SetMode(MANUAL);} // sets PID to manual
 
  
  
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
          indexup = 1;}}}                     // allows it to index. gets set back to 0 at end of sketch
      
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
      if(screenindex == 0) {
          screen++, lcd.clear();}}} // clears and indexes the screen if the backlight is on
          
  if (screenindex != lastindexstate) {
     backlightcount = millis();} // turns on the backlight using the button
     //notice the screen will not change unless the backlight is on

  /////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////
  //lcd logic printing



 //screen 0 //////////////////for changing proportional 
  if (screen == 0){                         // this is all pretty simple, lcd commands
    lcd.setCursor(0, 0);
    lcd.print("Proportional");
    lcd.setCursor(0, 1);
    lcd.print(consKp);
      if (indexdown == 1){consKp = (consKp-.01);}
      if (indexup == 1) {consKp = (consKp+.01);}
}
  
  // screen 1 //////////////// for changing integral
  if (screen == 1){
    lcd.setCursor(0, 0);
    lcd.print("Integral");
    lcd.setCursor(0, 1);
    lcd.print(consKi);
      if (indexdown == 1){consKi = (consKi-.01);}
      if (indexup == 1) {consKi = (consKi+.01);}
}
  
  // screen 2 /////////////// for changing derivative
  if (screen == 2){
    lcd.setCursor(0, 0);
    lcd.print("derivative");
    lcd.setCursor(0, 1);
    lcd.print(consKd);
      if (indexdown == 1){consKd = (consKd-.01);}
      if (indexup == 1) {consKd = (consKd+.01);}
}

   // screen 3 ////////////// for changing pid sample time////////
  if (screen == 3){
    lcd.setCursor(0, 0);
    lcd.print("PID Sample Time");
    lcd.setCursor(0, 1);
    lcd.print(pidsample);
      if (indexdown == 1){pidsample-= 100;}
      if (indexup == 1) {pidsample+= 100;}
}



// screen 4. asks if you want to load from epprom 
  if(screen==4){

 lcd.setCursor(0, 0);
    lcd.print("Hold up & down");
    lcd.setCursor(0, 1);
    lcd.print("to load Settings");
    
      if (upbuttonstate == LOW && downbuttonstate == LOW){// if both buttons are on
        //we use the button states here instead of the debounced indexers to get past the one or the other
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
  // screen 5
  if (screen == 5){
    lcd.setCursor(0, 0);
    lcd.print("Pit");
    lcd.print(Input);
    lcd.print("out");
    lcd.print(outputperc);
    lcd.setCursor(0, 1);
    lcd.print("Setpoint ");
    lcd.print(Setpoint);
    
    if (automanswitch == 0){// this section allows you to change the setpoint if
      if (indexdown == 1){Setpoint--;}  // the switch is set to auto
      if (indexup == 1) {Setpoint++;}}

    if (automanswitch == 1){          // this section allows you to adjust the output if and when
      if (indexdown == 1){Output-=2;} // the switch is set to manual
      if (indexup == 1) {Output+=2;}  // indexes by 2 to make it faster you can change this
    
      Output=constrain(Output,0,255); // constrains the output to the given values
       }}


    
  
 
  //screen 6 /////////////////// displays meat probe temps. lets you turn them on and off using the up down buttons
  if (screen == 6){
    lcd.setCursor(0, 0);
 if(alarm==0){       // if the alarm is off
    lcd.print("MeatProbesOn/Off");
    }
 if(alarm==1){       // if the alarm is on
    lcd.print("Foods Done!");
    }
    
    lcd.setCursor(0, 1);
      if (probestate>=1){  //here we decide what to print. based on if the probes are on or not
        lcd.print(meat1);
        }
      else{
        lcd.print("Off");
        }
    lcd.print(",");
    
      if (probestate>=2){ //probestate numbers indicate which probes are on 2 meaning probes 1 and 2 are on
        lcd.print(meat2);
        }
      else{
        lcd.print("Off");
        }
    lcd.print(",");
    
      if (probestate>=3){
        lcd.print(meat3);
        }
        else{
        lcd.print("Off");
        }
    if (indexdown == 1){probestate-=1;lcd.clear();alarm=0;}   // using the up down we can change probestate
    if (indexup == 1) {probestate+=1;lcd.clear();alarm=0;}
      probestate=constrain(probestate,0,3); //constrains the probestate value
    
  }
 
  // screen 7 //////////////////alarm temp setting for meat probes
  if (screen == 7){     //also lets us reset the alarm after the probe temp has fallen or has been turned off

  if (indexdown == 1){alarmtemp-=1;}// changes the alarmtemp down and turns the alarm off
    if (indexup == 1) {alarmtemp+=1;}
    
     alarmtemp=constrain(alarmtemp,100,300);// constrains

    lcd.setCursor(0, 0);
    
    lcd.print("MeatProbes Alarm");
      
    lcd.setCursor(0,1);
    lcd.print("Temp ");
    lcd.print(alarmtemp);

    lcd.setCursor(8, 1);    // below tells use which probes are turned on
      if (probestate>=1){
        lcd.print("#");
        lcd.print(1);}

    
      if (probestate>=2){ 
        lcd.print(" #");
        lcd.print(2);}

      if (probestate>=3){
        lcd.print(" #");
        lcd.print(3);}
    
    
  }

  
 
    
   // screen 8 used for eeprom saving /////////////
  if (screen == 8){         // just like eeprom reading
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


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
//fan output section uncomment this to use it////////////////////
/////////////////////////////////////////////////////////////////
/*

// after you have the PID setting set where you want them you can change the "screen =" number from 0 to 4 if you want

if (screen > 8){screen = 4;} // limits the screen number and starts it back to 0


//////////////////////////////////////////////////////////
  // output math
  outputperc=(Output/2.55); // makes a percentage to read on the lcd


  if ((millis() - prepulse) <= (Output*100)){ // a timer that pulses "on" based
    outpulse=1;}                         // on the output
    else {outpulse=0;}
    
  if ((millis() - prepulse) >25500){  // here, said timers off time is the remaining
    prepulse = millis();}             // amount withing the 255 bit range.
  
  
  if (outputperc>prange){analogWrite(11 ,Output);}    //if it's fast enough to pwm the fan, do it 
  
  if (outputperc<=prange){
      if(outpulse==1){digitalWrite(11 ,LOW);}  // if it's too slow, pulse it
  else{digitalWrite(11, HIGH);}
 }

  */
 /////////////////////////////////////////////////////////////////
 /////////////////////////////////////////////////////////////////
 //end of fan section////////////////////////////////////////////
 /////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
//Hobby servo  section uncomment this to use it////////////
//////////////////////////////////////////////////////////////
/*
 // after you have the PID setting set where you want them you can change the "screen =" number from 0 to 4 if you want
 
if (screen > 10){screen = 4;} // limits the screen number and starts it back to 0

   // screen 9 used for finding servo open position /////////////
  if (screen == 9){
    lcd.setCursor(0, 0);              // after you get these next two set change the screen number limit to 8
    lcd.print("Servo Open Pos");      // as you should not need it any more
    lcd.setCursor(0, 1);
    lcd.print(servofulopen);
      if (indexdown == 1){servofulopen--;}
      if (indexup == 1) {servofulopen++;}
      servofulopen=constrain(servofulopen,0,180);
      myservo.write(servofulopen);
}


   // screen 10 used for finding servo closed position //////////
  if (screen == 10){
    lcd.setCursor(0, 0);
    lcd.print("Servo Closed Pos");
    lcd.setCursor(0, 1);
    lcd.print(servofulclosed);
      if (indexdown == 1){servofulclosed--;}
      if (indexup == 1) {servofulclosed++;}
      servofulclosed=constrain(servofulclosed,0,180);
      myservo.write(servofulclosed);
}

 //////////////////////////////////////////////////////////
  // servo position math
  outputperc=(Output/2.55);
  servomath = (servofulopen - servofulclosed);
  servomath /= 100;
  servomath *= outputperc;
  servomath += servofulclosed;    // I tried doing all the math in one equation. it compiled 
 if (screen != 9 && screen != 10){
    myservo.write(servomath);       // but did not work. this works better than not working.
 }
  
*/
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
//end of servo stuff ///////////////////////////////////////////
///////////////////////////////////////////////////////////////




 /////////////////////////////////////////////////////////////
 /////////////////////////////////////////////////////////////
 // electric heating element section uncomment this to use it////////
 ////////////////////////////////////////////////////////////////////
 /*
// after you have the PID setting set where you want them you can change the "screen =" number from 0 to 4 if you want

if (screen > 9){screen = 0;} // limits the screen number and starts it back to 0
 
 
 // screen 9 used for setting the smoke element /////////////
  if (screen == 9){
    lcd.setCursor(0, 0);
    lcd.print("Smoke element");      
    lcd.setCursor(0, 1);
    lcd.print(smokeperc);
    lcd.print("%");
      if (indexdown == 1){smokeperc-=5;}   // if you want a more precise control you can turn down
      if (indexup == 1) {smokeperc+=5;}   // the incrementing values.
      
      smokeperc=constrain(smokeperc,0,100);
      
}

//////////////////////////////////////////////////////////
  // output math
  outputperc=(Output/2.55); // makes a percentage to read on the lcd

if (outputperc>=100){digitalWrite(11,relayon);}// a strange problem here. this is to keep
  else{       // the relays from chattering when the timer reset
    if ((millis() - prepulse) < (outputperc*100)){ // a timer that pulses "on" based
        digitalWrite(11 ,relayon);}                         // on the output
    else {digitalWrite(11, relayoff);}}

    
  if ((millis() - prepulse) >= 10000){  // here, said timer's off time is the remaining amount
    prepulse = millis();}             
  
//smoke element timer and output//////////////////////////////////////////////////////
if (smokeperc>=100){digitalWrite(13,smokeon);} //Same as above
  else{
        if ((millis() - smoketimer) < smokeperc*100){ // a timer that pulses "on" based
            digitalWrite(13 , smokeon);}                         // on the smoke setting
        else {digitalWrite(13 , smokeoff);}}

    
  if ((millis() - smoketimer) >= 10000){  // here, said timer's off time is the remaining amount
    smoketimer = millis();}     
*/
////////////////////////////////////////////////////////////////////////
/////////End of electric heating element section////////////////////
///////////////////////////////////////////////////////////////////////


    
 
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





//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
  ////thermistor function /////////////////////////////////////////////////////////


int tempRead(int THERMISTORPIN){




 uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(THERMISTORPIN);
   delay(10);
  }



  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;


  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;


  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
  fahrenheit =((steinhart*1.8)+32);
return fahrenheit;
}






