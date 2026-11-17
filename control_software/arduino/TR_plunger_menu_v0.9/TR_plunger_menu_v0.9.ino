#include <Servo.h> 
#include <LiquidCrystal_I2C.h> //SDA = B7[A4], SCL = B6[A5] STM32/[Arduino]
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Rotary Encoder Inputs
#define CLK 3
#define DT 2
#define SW 4

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

int counter = 1;
int currentStateCLK;
int lastStateCLK;
String currentDir ="";
unsigned long lastButtonPress = 0;
int menu = 0;

//LEDs
int LedTime1 = 50;
int LedTime2 = 100;
int LedTime3 = 250;
int LedTime4 = 500;

//Blot & Plunge
float BlotT = 1;
float BlotD = 27.1;
int ShutterOpen = 13;  
int ShutterClosed = 47; 
int PlungeArmUP = 0;
float PlungPOS = 123;
int PlungeMaxAcc = 210;
int PlungeMaxDec = PlungeMaxAcc;
float PlungeMaxVel = 2;
float PlungeTms;


 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  lcd.init();                         
  lcd.backlight();

  
  lcd.setCursor(0,0);
  lcd.print("     TR PLUNGER     ");
  lcd.setCursor(0,1);
  lcd.print("    INITIALIZING"    );
  
  // Set encoder pins as inputs
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  pinMode(SW, INPUT_PULLUP);

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
  
  // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT) != currentStateCLK) {
      counter ++;
    } else {
      // Encoder is rotating CW so increment
      counter --;
    }

    if (menu == 0) {
    // loops counter through 1 - 5 
    if (counter > 5) {counter = 1;}
    if (counter < 1) {counter = 5;}
    placecursorHome(counter);
    }

    
    if (menu == 1) {
    // loops counter through 1 - 8 
    if (counter > 8) {counter = 1;}
    if (counter < 1) {counter = 8;}
    placecursorSetup(counter);
    }
    

    if (menu == 2) {
    // loops counter through 1 - 8 (Setupmenu items?)
    if (counter > 5) {counter = 1;}
    if (counter < 1) {counter = 5;}
    placecursorPlungeSetupMenu(counter);
    }

    
  }
  
  // Remember last CLK state
  lastStateCLK = currentStateCLK;

  // Read the button state
  int btnState = digitalRead(SW);

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
    delay(250);

  if (VMenuSelected == 1) {Vcount; Vcount = LedTime1; vstep = 10;}
  if (VMenuSelected == 2) {Vcount = LedTime2; vstep = 10;}
  if (VMenuSelected == 3) {Vcount = LedTime3; vstep = 10;}
  if (VMenuSelected == 4) {Vcount = LedTime4; vstep = 10;}
  if (VMenuSelected == 5) {Vcount = BlotT; vstep = 0.1;}
  if (VMenuSelected == 6) {Vcount = BlotD; vstep = 0.1;}
      
    while(1 == 1){ 
        // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
    if (digitalRead(DT) != currentStateCLK) {
      Vcount = Vcount - vstep;
    } else {
      Vcount = Vcount + vstep;
    }
    //Serial.println(Vcount);

   if (VMenuSelected >0 && VMenuSelected < 5) {
    if (Vcount < 1) {Vcount = 0;}
    if (Vcount > 9990) {Vcount = 9990;}
   }

   if (VMenuSelected == 5) {
    if (Vcount < 0.1) {Vcount = 0;}
    if (Vcount > 99.9) {Vcount = 99.9;}
   }

   if (VMenuSelected == 6) {
    if (Vcount < 0.1) {Vcount = 0;}
    if (Vcount > 30) {Vcount = 30;}
   }

  if (VMenuSelected == 1) {LedTime1 = Vcount; lcd.setCursor(5,0); lcd.print("    "); lcd.setCursor(5,0); lcd.print(LedTime1);}
  if (VMenuSelected == 2) {LedTime2 = Vcount; lcd.setCursor(5,1); lcd.print("    "); lcd.setCursor(5,1); lcd.print(LedTime2);}
  if (VMenuSelected == 3) {LedTime3 = Vcount; lcd.setCursor(5,2); lcd.print("    "); lcd.setCursor(5,2); lcd.print(LedTime3);}
  if (VMenuSelected == 4) {LedTime4 = Vcount; lcd.setCursor(5,3); lcd.print("    "); lcd.setCursor(5,3); lcd.print(LedTime4);}
  if (VMenuSelected == 5) {BlotT = Vcount; lcd.setCursor(17,0); lcd.print("   "); lcd.setCursor(16,0); lcd.print(BlotT,1);}
  if (VMenuSelected == 6) {BlotD = Vcount; lcd.setCursor(17,1); lcd.print("   "); lcd.setCursor(16,1); lcd.print(BlotD,1); moveBlotter(BlotD);}  /// make this one live changing!
  }
  
  lastStateCLK = currentStateCLK;

  // Read the button state
  int btnState = digitalRead(SW);
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
  delay(2000);
  
  // shutter open (button) if not message shutter closed abort
  shutter.write(ShutterOpen);
  
  // tweezer to loading position (goto)
  //                                                               pos, maxvel, maxacc, maxdec
  goToWithParamsPlunger(message25plunger, sizeof(message25plunger), 10,      0.5,    1,    1); 
  Serial1.write(message25plunger, sizeof(message25plunger));
  PlungeArmUP = 0;
  counter = 2;
  placecursorHome(counter);
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 void RaizeTweezer(){
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

void RaizeEthane(){
  // tweezer up? raise lift to plunge postion
  if(PlungeArmUP == 1){
    //                                                       pos, maxvel, maxacc, maxdec
    goToWithParamsLift(message25lift, sizeof(message25lift), -5,      0.1,    0.5,    0.5); 
    Serial2.write(message25lift, sizeof(message25lift));
    delay(2000);
    
    }
  

  counter = 4;
  placecursorHome(counter);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BlotPlunge(){
  if(PlungeArmUP ==1) {
  // shutter closed?
  shutter.write(ShutterClosed);
  delay(50);
 
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
  LEDiluminate();
 // delay(1);

   //check sutter state and continue if shuttr is open
    
// if (digitalRead(ShutterControl) == HIGH){
 // delay(10);   //// -> IS THIS NEEDED? 
  //plunge (Go to with parameters)
  goToWithParamsPlunger(message25plunger, sizeof(message25plunger), PlungPOS, PlungeMaxVel, PlungeMaxAcc, PlungeMaxDec);
  Serial1.write(message25plunger, sizeof(message25plunger));
  delay(2000);

  //lower tweezer & lift simultaniously

  //                                                       pos, maxvel, maxacc, maxdec
  goToWithParamsLift(message25lift, sizeof(message25lift), 108,      0.1,    0.5,    0.5); 
  goToWithParamsPlunger(message25plunger, sizeof(message25plunger), 10,      0.1,    0.5,    0.5);
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


void LEDiluminate(){  
  
  int LEDarray[] = {LedTime1, LedTime2, LedTime3, LedTime4};
  int maxtime = 0; int L1 = 0; int L2 = 0; int L3 = 0; int L4 = 0;

  for (int i = 0; i < 4; i++){if ( LEDarray[i] > maxtime){maxtime = LEDarray[i];}}

  int LEDtrigger1 = maxtime - LedTime1;
  int LEDtrigger2 = maxtime - LedTime2;
  int LEDtrigger3 = maxtime - LedTime3;
  int LEDtrigger4 = maxtime - LedTime4;

  unsigned long TimeInit = millis();
 
  while(millis()<TimeInit + maxtime){
    if(millis() >= LEDtrigger1 + TimeInit && L1 == 0) {digitalWrite(LED1, HIGH); L1 = 1;}
    if(millis() >= LEDtrigger2 + TimeInit && L2 == 0) {digitalWrite(LED2, HIGH); L2 = 1;}
    if(millis() >= LEDtrigger3 + TimeInit && L3 == 0) {digitalWrite(LED3, HIGH); L3 = 1;}
    if(millis() >= LEDtrigger4 + TimeInit && L4 == 0) {digitalWrite(LED4, HIGH); L4 = 1;}  
  }

  digitalWrite(LED1, LOW); 
  digitalWrite(LED2, LOW); 
  digitalWrite(LED3, LOW); 
  digitalWrite(LED4, LOW); 
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
  if (vmenu == 0 && vcursor2 == 2){RaizeTweezer();}
  if (vmenu == 0 && vcursor2 == 3){RaizeEthane();}
  if (vmenu == 0 && vcursor2 == 4){BlotPlunge();}
  if (vmenu == 0 && vcursor2 == 5){setupMenu();}
  
  if (vmenu == 1 && vcursor2 == 1){ChangeValSetupMenu(vcursor2);}
  if (vmenu == 1 && vcursor2 == 2){ChangeValSetupMenu(vcursor2);}
  if (vmenu == 1 && vcursor2 == 3){ChangeValSetupMenu(vcursor2);}
  if (vmenu == 1 && vcursor2 == 4){ChangeValSetupMenu(vcursor2);}
  if (vmenu == 1 && vcursor2 == 5){ChangeValSetupMenu(vcursor2);}
  if (vmenu == 1 && vcursor2 == 6){ChangeValSetupMenu(vcursor2);}
  if (vmenu == 1 && vcursor2 == 7){plungeSetupMenu();}
  if (vmenu == 1 && vcursor2 == 8){plungeMenu();} 
  
  if (vmenu == 2 && vcursor2 == 1){ChangeValPlungeSetupMenu(vcursor2);}
  if (vmenu == 2 && vcursor2 == 2){ChangeValPlungeSetupMenu(vcursor2);}
  if (vmenu == 2 && vcursor2 == 3){ChangeValPlungeSetupMenu(vcursor2);}
  if (vmenu == 2 && vcursor2 == 4){ChangeValPlungeSetupMenu(vcursor2);}
  if (vmenu == 2 && vcursor2 == 5){plungeMenu();}
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
  lcd.setCursor (14,3); lcd.print(" ");
  if (vcursor == 1) { lcd.setCursor(0,0); lcd.print(">"); }
  if (vcursor == 2) { lcd.setCursor(0,1); lcd.print(">"); }
  if (vcursor == 3) { lcd.setCursor(0,2); lcd.print(">"); }
  if (vcursor == 4) { lcd.setCursor(0,3); lcd.print(">"); }
  if (vcursor == 5) { lcd.setCursor(14,3); lcd.print(">"); }
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setupMenu(){
  menu = 1;
  lcd.clear();
  lcd.setCursor (1,0);
  lcd.print("LD1:");
  lcd.print(LedTime1);
  lcd.setCursor (1,1);
  lcd.print("LD2:");
  lcd.print(LedTime2);
  lcd.setCursor (1,2);
  lcd.print("LD3:");
  lcd.print(LedTime3);
  lcd.setCursor (1,3);
  lcd.print("LD4:");
  lcd.print(LedTime4);
  lcd.setCursor (11,0);
  lcd.print("BltT:");
  lcd.print(BlotT,1);
  lcd.setCursor (11,1);
  lcd.print("BltD:");
  lcd.print(BlotD,1);
  lcd.setCursor (11,2);
  lcd.print("PlungeMen");
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
  lcd.print("Raize Tweezer");
  lcd.setCursor (1,2);
  lcd.print("Raize Ethane");
  lcd.setCursor (1,3);
  lcd.print("PLUNGE!");
 // lcd.setCursor (17,0);
 // lcd.print("menu5:");
 // lcd.setCursor (17,1);
 // lcd.print("menu6");
 // lcd.setCursor (17,2);
 // lcd.print("menu7:");
  lcd.setCursor (15,3);
  lcd.print("Setup");
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ChangeValPlungeSetupMenu(int VMenuSelected){
    float Vcount;
    float vstep;
    delay(250);

  if (VMenuSelected == 1) {Vcount; Vcount = PlungPOS; vstep = 0.1;}
  if (VMenuSelected == 2) {Vcount = PlungeMaxVel; vstep = 0.1;}
  if (VMenuSelected == 3) {Vcount = PlungeMaxAcc; vstep = 10;}
  if (VMenuSelected == 4) {Vcount = PlungeMaxDec; vstep = 10;}
      
    while(1 == 1){ 
        // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
    if (digitalRead(DT) != currentStateCLK) {
      Vcount = Vcount - vstep;
    } else {
      Vcount = Vcount + vstep;
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

  // Read the button state
  int btnState = digitalRead(SW);
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
