#include <Servo.h> 
#include <DHT.h>
#include <LiquidCrystal_I2C.h> //SDA = B7[A4], SCL = B6[A5] STM32/[Arduino]
LiquidCrystal_I2C lcd(0x27, 20, 4);
LiquidCrystal_I2C lcd2(0x24, 16, 2);

//Version 1.05
// Rotary Encoder Inputs
#define CLK 3
#define DT 2
#define SW 4

// relays
#define HumRelay 8
#define FanRelay 9

//Humidity/Temperature sensor
#define DHTPIN 31
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
float Temperature;
float Humidity = 50;
int SetHumidity = 20;
bool Fanstate = false;
bool Humstate = false;
unsigned long HumTimer = 0;
unsigned long HumCheckTimer = 0;
int HumCheckDelay = 200; //ms
  int switchdelay = 1000; // delay in ms between switching on/off humidifier

//LED Pins
const int LED1 = 10;
const int LED2 = 11;
const int LED3 = 12;
const int LED4 = 13;

//For serial comunication
bool nibblecounterPlunger = 1;
bool nibblecounterLift = 1;
//byte message13[13];
byte message25plunger[25];
byte message25lift[25];


//Servos
Servo shutter;
Servo blotter;
const int ShutterControl = 7;

// Joystick controls 
int xPin = A10;
int yPin = A9;
int joyButtonPin = 50;
int xVal;
int yVal;
int joyButtonState;
int lastXVal = 512;  // Neutral value
int joystickThreshold = 250;  // Sensitivity for movement
unsigned long lastMoveTime = 0;
unsigned long debounceDelay = 250; // ms

// CLK controls (old)
int counter = 1;
int currentStateCLK;
int lastStateCLK;
String currentDir ="";
unsigned long lastButtonPress = 0;
int menu = 0;

//LEDs
int LedTime1 = 0;
int LedTime2 = 0;
int LedTime3 = 7;
int LedTime4 = 50;
int earlyTriggerDelay = 6;

//Blot & Plunge
float BlotT = 8;
float BlotD = 27.5;
int ShutterOpen = 13;  
int ShutterClosed = 50; 
int PlungeArmUP = 0;
int LinkRaizeAndPlunge = 1;
float PlungPOS = 123;
int PlungeMaxAcc = 210;
int PlungeMaxDec = PlungeMaxAcc;
float PlungeMaxVel = 2;
float PlungeTms;

// Symbols
uint8_t tempsymbol[8] = {0x04, 0x06, 0x04, 0x06, 0x04, 0x0E, 0x0A, 0x0E};
uint8_t degreesymbol[8] = {  0x07,  0x05,  0x07,  0x00,  0x00,  0x00,  0x00,  0x00};
uint8_t humiditysymbol[8] = {0x04, 0x04, 0x04, 0x0E, 0x11, 0x11, 0x0A, 0x04};
uint8_t fansymbol[8] = { 0x1B, 0x1B, 0x0B, 0x06, 0x0C, 0x1A, 0x1B,  0x1B};
uint8_t humiditysetsymbol[] = {  0x04,  0x04,  0x04,  0x0E,  0x1F,  0x1F,  0x0E,  0x04};
 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  lcd.init();                         
  lcd.backlight();

  
  lcd.setCursor(0,0);
  lcd.print("  Optobot     ");
  lcd.setCursor(0,1);
  lcd.print("  version: 1");
  delay(2000);
  lcd.setCursor(0,1);
  lcd.print("    INITIALIZING");
  lcd2.init();                         
  lcd2.backlight(); 
  lcd2.createChar(0, tempsymbol);
  lcd2.createChar(1, degreesymbol);
  lcd2.createChar(2, humiditysymbol);
  lcd2.createChar(3, fansymbol);
  lcd2.createChar(4, humiditysetsymbol);
  //Print temp and humidity values
  lcd2.write(0);
  lcd2.setCursor(1,0);
  //lcd2.write(0); lcd2.print(Temperature,1); lcd2.write(1);lcd2.print("C");lcd2.print("  ");lcd2.write(2);lcd2.print(Humidity,1);lcd2.print("%");
  lcd2.write(0); lcd2.print("    "); lcd2.write(1);lcd2.print("C");lcd2.print(" ");lcd2.write(2);lcd2.print("    ");lcd2.print("%");
  lcd2.setCursor(1,1);
  lcd2.write(3);lcd2.print(":Off");lcd2.print("  "); lcd2.write(4);lcd2.print("Off "); lcd2.print("  <");//lcd2.print("Off");

  //Startup DHT
  dht.begin();

  //Start relays
  pinMode(HumRelay,OUTPUT);
  pinMode(FanRelay,OUTPUT);
  digitalWrite(HumRelay, LOW);
  digitalWrite(FanRelay, LOW);
  
  // Set encoder pins as inputs
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  pinMode(SW, INPUT_PULLUP);

  // Setup joystick pins 
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  pinMode(joyButtonPin, INPUT_PULLUP);

  // Set LED Pins
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  // Set servos & reset to 0 positions
  shutter.attach(5,530,2600);
  blotter.attach(6,530,2600);
  shutter.write(ShutterOpen); // -> Open position
  pinMode(ShutterControl, INPUT);
  moveBlotter(0);  // Retracted postion
  delay(100);
  //Calc time
  CalcTime();
  
  // Setup Serial Monitor for debugging only
 // Serial.begin(38400, SERIAL_8N1);

  // Read the initial state of CLK
  lastStateCLK = digitalRead(CLK);
  
  //Initialize plunger
  Serial1.begin(38400, SERIAL_8N1);
  //Initialize lift
  Serial2.begin(38400, SERIAL_8N1);
  //Startup lift & plunger
  startupPlungerAndLift();

  
  plungeMenu();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {
  
  // Read the current state of Joystick
  yVal = analogRead(yPin);
  unsigned long currentTime = millis();

  // Detect left or right movement
  if (currentTime - lastMoveTime > debounceDelay) {
    if (yVal < 512 - joystickThreshold) {
      counter ++;
      lastMoveTime = currentTime;
    } else if (yVal > 512 + joystickThreshold) {
      // Encoder is rotating CW so increment
      counter --;
      lastMoveTime = currentTime;
    }

    if (menu == 0) {
    // loops counter through 1 - 8 
    if (counter > 8) {counter = 8;}
    if (counter < 1) {counter = 1;}
    placecursorHome(counter);
    }

    
    if (menu == 1) {
    // loops counter through 1 - 8 
    if (counter > 8) {counter = 8;}
    if (counter < 1) {counter = 1;}
    placecursorSetup(counter);
    }
    

    if (menu == 2) {
    // loops counter through 1 - 8 (Setupmenu items?)
    if (counter > 5) {counter = 5;}
    if (counter < 1) {counter = 1;}
    placecursorPlungeSetupMenu(counter);
    }

  
    if (menu == 3) {
    // loops counter through 1 - 3 (Setupmenu items?)
    if (counter > 3) {counter = 3;}
    if (counter < 1) {counter = 1;}
    placecursorHumidifyMenu(counter);
    }
    
  }
  
  // Remember last CLK state
  lastStateCLK = currentStateCLK;

  // Read the button state
  int btnState = digitalRead(joyButtonPin);

  //If we detect LOW signal, button is pressed
  if (btnState == LOW) {
    //if 50ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 50) {
      menuSelection(counter,menu);
      
   //   Serial.println("Button pressed!");
    }

    // Remember last button press event
    lastButtonPress = millis();
  }

   // update humidity and Temperature every 200 ms? Can be faster
    if(millis() > HumCheckTimer + HumCheckDelay){
      //update vallues
      Temperature = dht.readTemperature();
      Humidity = dht.readHumidity();
      lcd2.setCursor(2,0); lcd2.print(Temperature,1);
      lcd2.setCursor(10,0); lcd2.print(Humidity,1);

      
      HumCheckTimer = millis();
      // Control humidifier
      HumidityControl();
    }


   
  


  // Put in a slight delay to help debounce the reading
  delay(1);
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Startup of linMot Plunger & lift
void startupPlungerAndLift() {
  lcd.setCursor(0,2);
  lcd.print("  Plunger and Lift: ");
  lcd.setCursor(0,3);
  lcd.print("  Deactivate Lock   ");
  byte message1[9] = {0x01,0x00,0x05,0x02,0x00,0x01,0x00,0x00,0x04};
  Serial1.write(message1, 9);
  delay(500);
  Serial2.write(message1, 9);
  delay(500);
  
  lcd.setCursor(0,3);
  lcd.print("       Homing       ");
  byte message2[9] = {0x01,0x00,0x05,0x02,0x00,0x01,0x3F,0x08,0x04};
  Serial2.write(message2, 9);
  delay(5000);
  Serial1.write(message2, 9);
  delay(4000);
  
  lcd.setCursor(0,3);
  lcd.print("  Normal Operation  ");
  byte message3[9] = {0x01,0x00,0x05,0x02,0x00,0x01,0x3F,0x00,0x04};
  Serial1.write(message3, 9);
  delay(1000);
  Serial2.write(message3, 9);
  delay(1000);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void goToWithParamsPlunger(byte * result, int size, float pos, float maxvel, float maxacc, float maxdec){
  if (pos>160) return;
  if (pos<0) return;
  unsigned long pos_acc = pos*10000;
  byte *pos_hx;
  pos_hx = (byte *)&pos_acc;
  
  unsigned long maxvel_acc = maxvel*1000000;
  byte *maxvel_hx;
  maxvel_hx = (byte *)&maxvel_acc;
  
  unsigned long maxacc_acc = maxacc*100000;
  byte *maxacc_hx;
  maxacc_hx = (byte *)&maxacc_acc;
  
  unsigned long maxdec_acc = maxdec*100000;
  byte *maxdec_hx;
  maxdec_hx = (byte *)&maxdec_acc;

  byte tmp[size] = {0x01,0x00,0x15,0x02,0x00,0x02,nibblecounterPlunger,0x01,
                       pos_hx[0],   pos_hx[1],   pos_hx[2],   pos_hx[3],
                    maxvel_hx[0],maxvel_hx[1],maxvel_hx[2],maxvel_hx[3],
                    maxacc_hx[0],maxacc_hx[1],maxacc_hx[2],maxacc_hx[3],
                    maxdec_hx[0],maxdec_hx[1],maxdec_hx[2],maxdec_hx[3], 
                    0x04};
  //01 11 15 02 00 02 03 01    F0 49 02 00    40 42 0F 00    40 42 0F 00    40 42 0F 00    04
  nibblecounterPlunger = !nibblecounterPlunger;
  for (unsigned i=0; i<size; i++)
  {  result[i] = tmp[i];  }  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void goToWithParamsLift(byte * result, int size, float pos, float maxvel, float maxacc, float maxdec){
  if (pos>150) return;
  if (pos<-5) return;
  unsigned long pos_acc = pos*10000;
  byte *pos_hx;
  pos_hx = (byte *)&pos_acc;
  
  unsigned long maxvel_acc = maxvel*1000000;
  byte *maxvel_hx;
  maxvel_hx = (byte *)&maxvel_acc;
  
  unsigned long maxacc_acc = maxacc*100000;
  byte *maxacc_hx;
  maxacc_hx = (byte *)&maxacc_acc;
  
  unsigned long maxdec_acc = maxdec*100000;
  byte *maxdec_hx;
  maxdec_hx = (byte *)&maxdec_acc;

  byte tmp[size] = {0x01,0x00,0x15,0x02,0x00,0x02,nibblecounterLift,0x01,
                       pos_hx[0],   pos_hx[1],   pos_hx[2],   pos_hx[3],
                    maxvel_hx[0],maxvel_hx[1],maxvel_hx[2],maxvel_hx[3],
                    maxacc_hx[0],maxacc_hx[1],maxacc_hx[2],maxacc_hx[3],
                    maxdec_hx[0],maxdec_hx[1],maxdec_hx[2],maxdec_hx[3], 
                    0x04};
  //01 11 15 02 00 02 03 01    F0 49 02 00    40 42 0F 00    40 42 0F 00    40 42 0F 00    04
  nibblecounterLift = !nibblecounterLift;
  for (unsigned i=0; i<size; i++)
  {  result[i] = tmp[i];  }  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ChangeValSetupMenu(int VMenuSelected){
    float Vcount;
    float vstep;
    unsigned long currentTime = millis();
    delay(250);

  if (VMenuSelected == 1) {Vcount; Vcount = LedTime1; vstep = 25;}
  if (VMenuSelected == 2) {Vcount = LedTime2; vstep = 10;}
  if (VMenuSelected == 3) {Vcount = LedTime3; vstep = 1;}
  if (VMenuSelected == 4) {Vcount = LedTime4; vstep = 10;}
  if (VMenuSelected == 5) {Vcount = BlotT; vstep = 0.1;}
  if (VMenuSelected == 6) {Vcount = BlotD; vstep = 0.1;}
  if (VMenuSelected == 7) {Vcount = LinkRaizeAndPlunge; vstep = 1;}

     
    while(1 == 1){ 
      currentTime = millis();
        // Read the current state of CLK
  xVal = analogRead(xPin);
  if (currentTime - lastMoveTime > debounceDelay) {
    if (xVal < 512 - joystickThreshold) {
      Vcount = Vcount + vstep;
      lastMoveTime = currentTime;
    } else if (xVal > 512 + joystickThreshold) {
      Vcount = Vcount - vstep;
      lastMoveTime = currentTime;
    }
    //Serial.println(Vcount);

   if (VMenuSelected >0 && VMenuSelected < 3 && VMenuSelected != 3) {
    if (Vcount < 1) {Vcount = 0;}
    if (Vcount > 9990) {Vcount = 9990;}
   }

   if (VMenuSelected == 3) {
    if (Vcount < 1) {Vcount = 0;}
    if (Vcount > 8) {Vcount = 8;}
   }

   if (VMenuSelected == 4) {
    if (Vcount < 10) {Vcount = 10;}
    if (Vcount > 9999) {Vcount = 9999;}
   }

   if (VMenuSelected == 5) {
    if (Vcount < 0.1) {Vcount = 0;}
    if (Vcount > 99.9) {Vcount = 99.9;}
   }

   if (VMenuSelected == 6) {
    if (Vcount < 0.1) {Vcount = 0;}
    if (Vcount > 30) {Vcount = 30;}
   }

    if (VMenuSelected == 7) {
      if (Vcount < 0) {Vcount = 0;}
     if (Vcount > 1) {Vcount = 1;}
   }

  if (VMenuSelected == 1) {LedTime1 = Vcount; lcd.setCursor(7,0); lcd.print("    "); lcd.setCursor(7,0); lcd.print(LedTime1);}
  if (VMenuSelected == 2) {LedTime2 = Vcount; lcd.setCursor(7,1); lcd.print("    "); lcd.setCursor(7,1); lcd.print(LedTime2);}
  if (VMenuSelected == 3) {LedTime3 = Vcount; lcd.setCursor(7,2); lcd.print("    "); lcd.setCursor(7,2); lcd.print(LedTime3);}
  if (VMenuSelected == 4) {LedTime4 = Vcount; lcd.setCursor(7,3); lcd.print("    "); lcd.setCursor(7,3); lcd.print(LedTime4);}
  if (VMenuSelected == 5) {BlotT = Vcount; lcd.setCursor(17,0); lcd.print("   "); lcd.setCursor(16,0); lcd.print(BlotT,1);}
  if (VMenuSelected == 6) {BlotD = Vcount; lcd.setCursor(17,1); lcd.print("   "); lcd.setCursor(16,1); lcd.print(BlotD,1); moveBlotter(BlotD);}  
  if (VMenuSelected == 7) {LinkRaizeAndPlunge = Vcount; lcd.setCursor(17,2); lcd.print("   "); lcd.setCursor(17,2); if(LinkRaizeAndPlunge == 1){lcd.print("Yes");};if(LinkRaizeAndPlunge == 0){lcd.print("No");}}
  }
  
  lastStateCLK = currentStateCLK;

   // update humidity and Temperature every 200 ms? Can be faster
    if(millis() > HumCheckTimer + HumCheckDelay){
      //update vallues
      Temperature = dht.readTemperature();
      Humidity = dht.readHumidity();;
      HumCheckTimer = millis();
      lcd2.setCursor(2,0); lcd2.print(Temperature,1);
      lcd2.setCursor(10,0); lcd2.print(Humidity,1);
      // Control humidifier
      HumidityControl();
    }
  
  // Read the button state
  int btnState = digitalRead(joyButtonPin);
  if (btnState == LOW) {
    if (millis() - lastButtonPress > 50) {
      
      if (VMenuSelected ==6) {moveBlotter(0);}
      break;
    }
    lastButtonPress = millis();
    
    }
} 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LoadTweezer(){
  // lift down
  //                                                       pos, maxvel, maxacc, maxdec
  goToWithParamsLift(message25lift, sizeof(message25lift), 150,      0.1,    0.5,    0.5); 
  Serial2.write(message25lift, sizeof(message25lift));
  delay(100);
  
  // shutter open (button) if not message shutter closed abort
  shutter.write(ShutterOpen);
  
  // tweezer to loading position (goto)
  //                                                               pos, maxvel, maxacc, maxdec
  goToWithParamsPlunger(message25plunger, sizeof(message25plunger), 10,      0.5,    1,    1); 
  Serial1.write(message25plunger, sizeof(message25plunger));
  delay(1800);
  
  PlungeArmUP = 0;
  counter = 2;
  placecursorHome(counter);
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 void RaiseTweezer(){
  // shutter open (button) if not message shutter clsed abort
  shutter.write(ShutterOpen);
  // Tweezer to plunge postion
  //                                                                 pos, maxvel, maxacc, maxdec
      goToWithParamsPlunger(message25plunger, sizeof(message25plunger), 160,      0.5,    1,    1);
      Serial1.write(message25plunger, sizeof(message25plunger));
      delay(2000);
  // shutter closed
  shutter.write(ShutterClosed);
  delay(500);
  PlungeArmUP = 1;
  counter = 3;
  placecursorHome(counter);

 }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaiseEthane(){
  // Increase humidity before blotting
  digitalWrite(HumRelay, HIGH);
  delay(5000); // give  seconds time to set the humidity to 100%
  // tweezer up? Raise lift to plunge postion
  if(PlungeArmUP == 1){
    //                                                       pos, maxvel, maxacc, maxdec
    goToWithParamsLift(message25lift, sizeof(message25lift), -5,      0.1,    0.5,    0.5); 
    Serial2.write(message25lift, sizeof(message25lift));
    delay(800);

    if(LinkRaizeAndPlunge == 1){Fanstate = false; digitalWrite(FanRelay, LOW);digitalWrite(HumRelay, LOW);}   /// fan off, humidity off
    
    delay(1200);
    counter = 4;
    placecursorHome(counter);

    if(LinkRaizeAndPlunge == 1){BlotPlunge();}   /// Plunge
    
    }
   
   
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BlotPlunge(){
  if(PlungeArmUP ==1) {
  // shutter closed?
  shutter.write(ShutterClosed);
  delay(50);
   
   Fanstate = false; 
   digitalWrite(FanRelay, LOW);
   digitalWrite(HumRelay, LOW);
  //Blot
  moveBlotter(BlotD);
   delay(BlotT * 1000);
   moveBlotter(BlotD);
 
  //shutter & blotter rectractopen
  shutter.write(ShutterOpen);
  //delay(50);
  moveBlotter(0);
  delay(50);

  //Iluminate
  LEDilluminateAndPlunge();


  delay(2000);

  //lower tweezer & lift simultaniously

  //                                                       pos, maxvel, maxacc, maxdec
  goToWithParamsLift(message25lift, sizeof(message25lift), 118,      0.1,    0.5,    0.5); 
  goToWithParamsPlunger(message25plunger, sizeof(message25plunger), 0,      0.1,    0.5,    0.5);
  Serial2.write(message25lift, sizeof(message25lift));
  Serial1.write(message25plunger, sizeof(message25plunger));
  delay(2000);

  PlungeArmUP = 0;
  counter = 1;
  placecursorHome(counter);
  
 // } // activiate this one for controlbutton 
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void plungeNow(){
  goToWithParamsPlunger(message25plunger, sizeof(message25plunger), PlungPOS, PlungeMaxVel, PlungeMaxAcc, PlungeMaxDec);
  Serial1.write(message25plunger, sizeof(message25plunger));
}

void LEDilluminateAndPlunge(){   
 // int LEDarray[] = {LedTime1, LedTime2, LedTime3, LedTime4};
  
  delay(100);
  bool plungingDone = false;
  unsigned long TotalTime;
  unsigned long PlungeTime;
  unsigned long FlashTime;
  unsigned long TimeInit = millis();
  FlashTime = TimeInit + LedTime4;
  TotalTime = FlashTime + LedTime1; // LedTime1 = Waiting Time
  PlungeTime = TotalTime - LedTime3; // LedTime3 = early Trigger Delay
  

  // Start flash
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);
  digitalWrite(LED4, HIGH);
  while(millis()< TotalTime){
    if(millis() >= PlungeTime && plungingDone == false) {plungeNow();plungingDone=true;}
    if(millis() >= FlashTime) {digitalWrite(LED1, LOW);digitalWrite(LED2, LOW);digitalWrite(LED3, LOW);digitalWrite(LED4, LOW);}  
  }

  digitalWrite(LED1, LOW); 
  digitalWrite(LED2, LOW); 
  digitalWrite(LED3, LOW); 
  digitalWrite(LED4, LOW); 
}


void LEDiluminate(){  
  
 // int LEDarray[] = {LedTime1, LedTime2, LedTime3, LedTime4};
    int LEDarray[] = {LedTime2, LedTime3, LedTime4};
  
  int maxtime = 0; int L1 = 0; int L2 = 0; int L3 = 0; int L4 = 0;

  for (int i = 0; i < 3; i++){if ( LEDarray[i] > maxtime){maxtime = LEDarray[i];}}

  //int LEDtrigger1 = maxtime - LedTime1;
  int LEDtrigger2 = maxtime - LedTime2;
  int LEDtrigger3 = maxtime - LedTime3;
  int LEDtrigger4 = maxtime - LedTime4;
  
  if(LEDtrigger4 <=49 ){delay(50);}
  unsigned long TimeInit = millis();

 
 
  while(millis()<TimeInit + maxtime){
   // if(millis() >= LEDtrigger1 + TimeInit && L1 == 0) {digitalWrite(LED1, HIGH); L1 = 1;}
    if(millis() >= LEDtrigger2 + TimeInit && L2 == 0) {digitalWrite(LED2, HIGH); L2 = 1;}
    if(millis() >= LEDtrigger3 + TimeInit && L3 == 0) {digitalWrite(LED3, HIGH); L3 = 1;}
    if(millis() >= LEDtrigger4 + TimeInit && L4 == 0) {digitalWrite(LED4, HIGH); L4 = 1;}  
  }

  digitalWrite(LED1, LOW); 
  digitalWrite(LED2, LOW); 
  digitalWrite(LED3, LOW); 
  digitalWrite(LED4, LOW); 
  delay(LedTime1);
  //return;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void moveBlotter(float mmPosition) {
  // 0 mm = 120 degrees
  // max 30 mm = 0 degrees
  // mm/degrees = 
  // 
  
  float degreesPosition = -4 * mmPosition + 120;  
  blotter.write(degreesPosition);

  
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void menuSelection(int vcursor2, int vmenu){
  if (vmenu == 0 && vcursor2 == 1){LoadTweezer();}
  if (vmenu == 0 && vcursor2 == 2){RaiseTweezer();}
  if (vmenu == 0 && vcursor2 == 3){RaiseEthane();}
  if (vmenu == 0 && vcursor2 == 4){BlotPlunge();}
  if (vmenu == 0 && vcursor2 == 5){setupMenu();}
  if (vmenu == 0 && vcursor2 == 6){plungeSetupMenu();}
  if (vmenu == 0 && vcursor2 == 7){HumMenu();}
  if (vmenu == 0 && vcursor2 == 8){shutdownPlunger();}

  if (vmenu == 1 && vcursor2 == 1){ChangeValSetupMenu(vcursor2);}
  if (vmenu == 1 && vcursor2 == 2){ChangeValSetupMenu(vcursor2);}
  if (vmenu == 1 && vcursor2 == 3){ChangeValSetupMenu(vcursor2);}
  if (vmenu == 1 && vcursor2 == 4){ChangeValSetupMenu(vcursor2);}
  if (vmenu == 1 && vcursor2 == 5){ChangeValSetupMenu(vcursor2);}
  if (vmenu == 1 && vcursor2 == 6){ChangeValSetupMenu(vcursor2);}
  if (vmenu == 1 && vcursor2 == 7){ChangeValSetupMenu(vcursor2);}
  if (vmenu == 1 && vcursor2 == 8){plungeMenu();} //home
  
  if (vmenu == 2 && vcursor2 == 1){ChangeValPlungeSetupMenu(vcursor2);}
  if (vmenu == 2 && vcursor2 == 2){ChangeValPlungeSetupMenu(vcursor2);}
  if (vmenu == 2 && vcursor2 == 3){ChangeValPlungeSetupMenu(vcursor2);}
  if (vmenu == 2 && vcursor2 == 4){ChangeValPlungeSetupMenu(vcursor2);}
  if (vmenu == 2 && vcursor2 == 5){plungeMenu();} //home

  if (vmenu == 3 && vcursor2 == 1){Setfanstate();}
  if (vmenu == 3 && vcursor2 == 2){ChangeValHumidifyMenu(vcursor2);}
  if (vmenu == 3 && vcursor2 == 3){lcd2.setCursor (14,1); lcd2.print(" "); plungeMenu();}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Setfanstate(){

  if(Fanstate == false){
    Fanstate = true;
    digitalWrite(FanRelay, HIGH);
    lcd2.setCursor(3,1);
    lcd2.print("On ");
    }
  
  else if(Fanstate == true){
    Fanstate = false;
    digitalWrite(FanRelay, LOW);
    lcd2.setCursor(3,1);
    lcd2.print("Off"); 
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void HumidityControl(){
 
  if(Fanstate == false && SetHumidity > 20){
    Fanstate = true;
    digitalWrite(FanRelay, HIGH);
    lcd2.setCursor(3,1);
    lcd2.print("On ");
    }
  
  if(millis() > HumTimer + switchdelay){
  
    if(Humidity < SetHumidity){
      digitalWrite(HumRelay, HIGH);
      HumTimer = millis();
    }
  
    else if (Humidity >= SetHumidity){
      digitalWrite(HumRelay, LOW);
      HumTimer = millis();
    }
  }
  
  }


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void placecursorSetup(int vcursor){
  lcd.setCursor (0,0);  lcd.print(" ");
  lcd.setCursor (0,1);  lcd.print(" ");
  lcd.setCursor (0,2);  lcd.print(" ");
  lcd.setCursor (0,3);  lcd.print(" ");
  lcd.setCursor (10,0); lcd.print(" ");
  lcd.setCursor (10,1); lcd.print(" ");
  lcd.setCursor (10,2); lcd.print(" ");
  lcd.setCursor (10,3); lcd.print(" ");
  if (vcursor == 1) { lcd.setCursor(0,0); lcd.print(">"); }
  if (vcursor == 2) { lcd.setCursor(0,1); lcd.print(">"); }
  if (vcursor == 3) { lcd.setCursor(0,2); lcd.print(">"); }
  if (vcursor == 4) { lcd.setCursor(0,3); lcd.print(">"); }
  if (vcursor == 5) { lcd.setCursor(10,0); lcd.print(">"); }
  if (vcursor == 6) { lcd.setCursor(10,1); lcd.print(">"); }
  if (vcursor == 7) { lcd.setCursor(10,2); lcd.print(">"); }
  if (vcursor == 8) { lcd.setCursor(10,3); lcd.print(">"); }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void placecursorHome(int vcursor){
  lcd.setCursor (0,0);  lcd.print(" ");
  lcd.setCursor (0,1);  lcd.print(" ");
  lcd.setCursor (0,2);  lcd.print(" ");
  lcd.setCursor (0,3);  lcd.print(" ");
  lcd.setCursor (14,0); lcd.print(" ");
  lcd.setCursor (14,1); lcd.print(" ");
  lcd.setCursor (14,2); lcd.print(" ");
  lcd.setCursor (14,3); lcd.print(" ");
  if (vcursor == 1) { lcd.setCursor(0,0); lcd.print(">"); }
  if (vcursor == 2) { lcd.setCursor(0,1); lcd.print(">"); }
  if (vcursor == 3) { lcd.setCursor(0,2); lcd.print(">"); }
  if (vcursor == 4) { lcd.setCursor(0,3); lcd.print(">"); }
  if (vcursor == 5) { lcd.setCursor(14,0); lcd.print(">"); }
  if (vcursor == 6) { lcd.setCursor(14,1); lcd.print(">"); }
  if (vcursor == 7) { lcd.setCursor(14,2); lcd.print(">"); }
  if (vcursor == 8) { lcd.setCursor(14,3); lcd.print(">"); }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void placecursorPlungeSetupMenu(int vcursor){
  lcd.setCursor (0,0);  lcd.print(" ");
  lcd.setCursor (11,0);  lcd.print(" ");
  lcd.setCursor (0,1);  lcd.print(" ");
  lcd.setCursor (11,1);  lcd.print(" ");
  lcd.setCursor (0,2); lcd.print(" ");
    if (vcursor == 1) { lcd.setCursor(0,0); lcd.print(">"); }
  if (vcursor == 2) { lcd.setCursor(11,0); lcd.print(">"); }
  if (vcursor == 3) { lcd.setCursor(0,1); lcd.print(">"); }
  if (vcursor == 4) { lcd.setCursor(11,1); lcd.print(">"); }
  if (vcursor == 5) { lcd.setCursor(0,2); lcd.print(">"); }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void placecursorHumidifyMenu(int vcursor){
  lcd2.setCursor (0,1);  lcd2.print(" "); lcd2.setCursor (7,1); lcd2.print(" "); lcd2.setCursor (14,1); lcd2.print(" ");
  if (vcursor == 1) { lcd2.setCursor(0,1); lcd2.print(">"); }
  if (vcursor == 2) { lcd2.setCursor(7,1); lcd2.print(">"); }
  if (vcursor == 3) { lcd2.setCursor(14,1); lcd2.print(">"); }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void HumMenu(){
  menu = 3;
  lcd.setCursor (14,2); lcd.print(" ");
  counter = 1;
  placecursorHumidifyMenu(counter);
 }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setupMenu(){
  menu = 1;
  lcd.clear();
  lcd.setCursor (1,0);
  lcd.print("WaitT:");
  lcd.print(LedTime1);
  lcd.setCursor (1,1);
  lcd.print("LED_2:");
  lcd.print(LedTime2);
  lcd.setCursor (1,2);
  lcd.print("PreTr:");
  lcd.print(LedTime3);
  lcd.setCursor (1,3);
  lcd.print("Pulse:");
  lcd.print(LedTime4);
  lcd.setCursor (11,0);
  lcd.print("BltT:");
  lcd.print(BlotT,1);
  lcd.setCursor (11,1);
  lcd.print("BltD:");
  lcd.print(BlotD,1);
  lcd.setCursor (11,2);
  lcd.print("RE&PL:");
  if(LinkRaizeAndPlunge == 1){lcd.print("Yes");}
  if(LinkRaizeAndPlunge == 0){lcd.print("No");}
  lcd.setCursor (11,3);
  lcd.print("Home");
  counter = 1;
  placecursorSetup(counter);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void plungeMenu(){
  menu = 0;
  lcd.clear();
  lcd.setCursor (1,0);
  lcd.print("Load Tweezer");
  lcd.setCursor (1,1);
  lcd.print("Raise Tweezer");
  lcd.setCursor (1,2);
  lcd.print("Raise Ethane");
  lcd.setCursor (1,3); 
  lcd.print("PLUNGE!");
  lcd.setCursor (15,0);
  lcd.print("BLMen");
  lcd.setCursor (15,1);
  lcd.print("PLMen"); 
  lcd.setCursor (15,2);
  lcd.print("Humid");
  lcd.setCursor (15,3);
  lcd.print("Exit");
  counter = 1;
  placecursorHome(counter);  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void plungeSetupMenu(){
  menu = 2;
  lcd.clear();
  lcd.setCursor (1,0);
  lcd.print("POS:");
  lcd.print(PlungPOS,1);

  lcd.setCursor (12,0);
  lcd.print("Vel:");
  lcd.print(PlungeMaxVel,1);
  
  lcd.setCursor (1,1);
  lcd.print("Acc:");
  lcd.print(PlungeMaxAcc);
  
  lcd.setCursor (12,1);
  lcd.print("Dec:");
  lcd.print(PlungeMaxDec);
  
  lcd.setCursor (1,2);
  lcd.print("Home");
  
  lcd.setCursor (1,3);
  lcd.print("CalcTime (ms):");
  lcd.print(PlungeTms,1);

  counter = 1;
  placecursorPlungeSetupMenu(counter); 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void ChangeValHumidifyMenu(int VMenuSelected){
    float Vcount;
    float vstep;
    unsigned long currentTime = millis();
    delay(250);

  if (VMenuSelected == 2) {Vcount = SetHumidity; vstep = 1;}
  
      
    while(1 == 1){ 
      currentTime = millis();
        // Read the current state of CLK
      xVal = analogRead(xPin);
  if (currentTime - lastMoveTime > debounceDelay) {
    if (xVal < 512 - joystickThreshold) {
      Vcount = Vcount + vstep;
      lastMoveTime = currentTime;
    } else if (xVal > 512 + joystickThreshold) {
      Vcount = Vcount - vstep;
      lastMoveTime = currentTime;
    }
    //Serial.println(Vcount);

//standard 123 range +-5
   if (VMenuSelected == 2) {
    if (Vcount < 20) {Vcount = 20;}
    if (Vcount > 100) {Vcount = 100;}
   }

  if (VMenuSelected == 2) {SetHumidity = Vcount; 
    lcd2.setCursor(9,1); lcd2.print("   ");
    
    if(SetHumidity > 20){lcd2.setCursor(9,1); lcd2.print(SetHumidity);lcd2.print("%");}
    if(SetHumidity == 20){lcd2.setCursor(9,1); lcd2.print("Off "); Fanstate = false; lcd2.setCursor(3,1); lcd2.print("Off");digitalWrite(FanRelay, LOW);}
    }

    
    }
  
  lastStateCLK = currentStateCLK;

    // update humidity and Temperature every 200 ms? Can be faster
    if(millis() > HumCheckTimer + HumCheckDelay){
      //update vallues
      Temperature = dht.readTemperature();
      Humidity = dht.readHumidity();
      lcd2.setCursor(2,0); lcd2.print(Temperature,1);
      lcd2.setCursor(10,0); lcd2.print(Humidity,1);
      HumCheckTimer = millis();
      // Control humidifier
      HumidityControl();
    }
      
  // Read the button state
  int btnState = digitalRead(joyButtonPin);
  if (btnState == LOW) {
    if (millis() - lastButtonPress > 50) {
      break;
    }
    lastButtonPress = millis();
    
    }
} 
}






//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ChangeValPlungeSetupMenu(int VMenuSelected){
    float Vcount;
    float vstep;
    unsigned long currentTime;
    delay(250);

  if (VMenuSelected == 1) {Vcount; Vcount = PlungPOS; vstep = 0.1;}
  if (VMenuSelected == 2) {Vcount = PlungeMaxVel; vstep = 0.1;}
  if (VMenuSelected == 3) {Vcount = PlungeMaxAcc; vstep = 10;}
  if (VMenuSelected == 4) {Vcount = PlungeMaxDec; vstep = 10;}
      
    while(1 == 1){ 
      currentTime = millis();
        // Read the current state of CLK
  xVal = analogRead(xPin);
  if (currentTime - lastMoveTime > debounceDelay) {
    if (xVal < 512 - joystickThreshold) {
      Vcount = Vcount + vstep;
      lastMoveTime = currentTime;
    } else if (xVal > 512 + joystickThreshold) {
      Vcount = Vcount - vstep;
      lastMoveTime = currentTime;
    }
    //Serial.println(Vcount);

//standard 123 range +-5
   if (VMenuSelected == 1) {
    if (Vcount < 118) {Vcount = 118;}
    if (Vcount > 128) {Vcount = 128;}
   }

    if (VMenuSelected == 2) {
    if (Vcount < 0.1) {Vcount = 0.1;}
    if (Vcount > 10) {Vcount = 10;}
   }

   if (VMenuSelected > 2 && VMenuSelected < 5) {
    if (Vcount < 1) {Vcount = 1;}
    if (Vcount > 650) {Vcount = 650;}
   }

  if (VMenuSelected == 1) {PlungPOS = Vcount; CalcTime(); lcd.setCursor(5,0); lcd.print("     "); lcd.setCursor(5,0); lcd.print(PlungPOS,1); lcd.setCursor(15,3); lcd.print("     "); lcd.setCursor(15,3); lcd.print(PlungeTms,1); }
  if (VMenuSelected == 2) {PlungeMaxVel = Vcount; CalcTime(); lcd.setCursor(16,0); lcd.print("    "); lcd.setCursor(16,0); lcd.print(PlungeMaxVel,1);  lcd.setCursor(15,3); lcd.print("     "); lcd.setCursor(15,3); lcd.print(PlungeTms,1);}
  if (VMenuSelected == 3) {PlungeMaxAcc = Vcount; PlungeMaxDec = PlungeMaxAcc; CalcTime(); lcd.setCursor(5,1); lcd.print("    "); lcd.setCursor(5,1); lcd.print(PlungeMaxAcc);lcd.setCursor(16,1); lcd.print("    "); lcd.setCursor(16,1); lcd.print(PlungeMaxDec);  lcd.setCursor(15,3); lcd.print("     "); lcd.setCursor(15,3); lcd.print(PlungeTms,1);}
  if (VMenuSelected == 4) {PlungeMaxDec = Vcount; PlungeMaxAcc = PlungeMaxDec; CalcTime(); lcd.setCursor(16,1); lcd.print("    "); lcd.setCursor(16,1); lcd.print(PlungeMaxDec); lcd.setCursor(5,1); lcd.print("    "); lcd.setCursor(5,1); lcd.print(PlungeMaxAcc);  lcd.setCursor(15,3); lcd.print("     "); lcd.setCursor(15,3); lcd.print(PlungeTms,1);}
    }
  
  lastStateCLK = currentStateCLK;

   // update humidity and Temperature every 200 ms? Can be faster
    if(millis() > HumCheckTimer + HumCheckDelay){
      //update vallues
      Temperature = dht.readTemperature();
      Humidity = dht.readHumidity();;
      HumCheckTimer = millis();
      // Control humidifier
      HumidityControl();
    }
    
  // Read the button state
  int btnState = digitalRead(joyButtonPin);
  if (btnState == LOW) {
    if (millis() - lastButtonPress > 50) {
      break;
    }
    lastButtonPress = millis();
    
    }
} 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void CalcTime(){
 float dist = (160 - PlungPOS) / 1000;  // in meter
 float TimetoMaxVel = PlungeMaxVel / PlungeMaxAcc;
 float DisttoMaxVel = 0.5*PlungeMaxAcc * TimetoMaxVel * TimetoMaxVel;

  //// at the moment maxACC == MacDec
 if(DisttoMaxVel < dist/2){PlungeTms = (((dist - 2 * DisttoMaxVel) / PlungeMaxVel) + 2 * TimetoMaxVel)*1000;}
 
 if(DisttoMaxVel >= dist/2){PlungeTms = 2 * sqrt(2*(dist/2)/PlungeMaxAcc) * 1000;}

}

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void shutdownPlunger(){
  shutter.write(ShutterOpen);
  moveBlotter(0);
  delay(100);

  // lift down
  goToWithParamsLift(message25lift, sizeof(message25lift), 150,      0.1,    0.5,    0.5); 
  Serial2.write(message25lift, sizeof(message25lift));
  delay(2000);

    //                                                               pos, maxvel, maxacc, maxdec
  goToWithParamsPlunger(message25plunger, sizeof(message25plunger), 0,      0.5,    1,    1); 
  Serial1.write(message25plunger, sizeof(message25plunger));
  PlungeArmUP = 0;

   SetHumidity = 20;
   lcd2.setCursor(9,1); 
   lcd2.print("Off ");
   digitalWrite(HumRelay, LOW);
   Fanstate = false; 
   digitalWrite(FanRelay, LOW);

  digitalWrite(LED1, LOW); 
  digitalWrite(LED2, LOW); 
  digitalWrite(LED3, LOW); 
  digitalWrite(LED4, LOW); 
  delay(2000);

  lcd.clear();
  lcd2.clear();
  lcd.noBacklight();
  lcd2.noBacklight();

}
