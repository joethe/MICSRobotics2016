 #include <AbsoluteIMU.h>
#include <ACCLNx.h>
#include <AngleSensor.h>
#include <BaseI2CDevice.h>
#include <DISTNx.h>
#include <EV3Color.h>
#include <EV3Gyro.h>
#include <EV3InfraRed.h>
#include <EV3SensorAdapter.h>
#include <EV3Sonar.h>
#include <LineLeader.h>
#include <NXShield.h>
#include <NXShieldAGS.h>
#include <NXShieldI2C.h>
#include <NXTCam.h>
#include <NXTHID.h>
#include <NXTLight.h>
#include <NXTPowerMeter.h>
#include <NXTServo.h>
#include <NXTTouch.h>
#include <NXTUS.h>
#include <NXTVoltMeter.h>
#include <PFMate.h>
#include <PSPNx.h>
#include <RTC.h>
#include <SoftI2cMaster.h>
#include <SumoEyes.h>
#include <Wire.h>
#include <Servo.h>
#include <NXShield.h>
#include <NXTUS.h>
#include <NXTMMX.h>

#include <SoftwareSerial.h>
//Don't make # symbols

char buffer [32];
int bIndex = 0;
boolean ready = false;

///////////////////
// Config Values //
///////////////////
const int UPDATE_DELAY = 50;

// The shield
NXShield nxshield;

//The SERVO with the vision camera on it
Servo cam;

// Declare the i2c devices used on NXShield(s).
NXTMMX      mmx(0x0A); // Multiplexer for chain morors

// 2 line LCD on arduino digital pin #6
SoftwareSerial lcd(2, 5);

//Battery Global//
float batVolt = 420.0; //if you see this printed, some dank shit's going on.

///
/////////// SETUP ///////////////////////////////////////////////
///

void setup(){
  //SERIAL//
  Serial.begin(115200);
  Serial.print("Setup Starting...");

  //LCD//
  lcd.begin(9600);
  lcd.print("Setup Starting...");
  delay(500);

  //PIN//
  pinMode(6, OUTPUT);
  pinMode(3, INPUT_PULLUP);
  cam.attach(9);

  //NX SHIT//
  nxshield.init(SH_HardwareI2C);
  nxshield.bank_a.motorReset();
  nxshield.bank_b.motorReset();

  // Check battery voltage on startup. Warn if low.
  
  
  while((digitalRead(3) == HIGH) & (checkBattery())) {
    delay(500);
    }
    
  clearDisplay();
  lcd.print("Press GO!");
  setLCDCursor(16);
  lcd.print("voltage: ");
  lcd.print(batVolt);
  delay(500);
  
 // Initialize the i2c sensors.//
 // OK LOL

 // sensor diagnostic mode. //
  if(nxshield.getButtonState(BTN_RIGHT)){
     clearDisplay();
     beep();
     lcd.print("Diagnostic Mode!");
     delay(5000);
     while(true){
       delay(UPDATE_DELAY);
     }
  } 
// end diagnostics mode //

  Serial.print("Setup Complete...");
  clearDisplay();

} // end setup



boolean checkBattery(){
  batVolt = (float) nxshield.bank_a.nxshieldGetBatteryVoltage() / 1000;
  boolean goodBatt = (batVolt > 7.50);
  if(!goodBatt) {
    clearDisplay();
    beep();
    Serial.println("Low Voltage!");
    Serial.println(batVolt);
    lcd.print("LOW VOLTAGE!!!");
    setLCDCursor(16);
    lcd.print("voltage: ");
    lcd.print(batVolt);
  }
  return(goodBatt);
}
////////////////////////
      ////////////
///////// LOOP /////////
      ////////////
////////////////////////

//void loop(){
//  cam.write(125);
//  delay(2000);
//  cam.write(90);
//  delay(2000);
//
//}
void loop(){

  if(ready){
    processCommand();
    ready = false;
  } else {
      while(Serial.available()){
        char c = Serial.read();
        buffer[bIndex++] = c;

        if ((c == '\n') || (bIndex == sizeof(buffer) - 1)){
          buffer[bIndex] = '\0';
          bIndex = 0;
          ready = true;
        }
      }
  }

}


///
/////////// Helper Functions ///////////////////////////////////////////////
///

void processCommand(){
  //char *cmd;
  //char *direction;
  //char *speed;
  //char *degrees;
  String cmd;
  String dir;
  String spd;
  String deg;

  Serial.print("Processing Command:");
  Serial.print(buffer);

  cmd = strtok(buffer, " ");
  spd = strtok(NULL, " ");
  deg = strtok(NULL, " ");
  dir = strtok(NULL, " ");

  Serial.print("CMD: ");
  Serial.print(cmd);

  int spd_ = spd.toInt();
  long deg_ = StrToFloat(deg);
  int dir_ = dir.toInt();

  if(cmd == "FWD"){
    forward(spd_, deg_);
  } else if(cmd == "REV"){
    reverse(spd_, deg_);
  } else if(cmd == "SPN"){
    rotate(dir_, spd_, deg_);
  } else if(cmd == "HIT"){
      hitter(spd_, deg_, dir_);
  } else if(cmd == "LFT"){
      leftDrive(dir_, spd_, deg_);
  } else if(cmd == "RHT"){
      rightDrive(dir_, spd_, deg_);
  } else if(cmd == "CUP"){//Camera UP
      cam.write(90);
  } else if(cmd == "CDN"){//Camera DowN
      cam.write(125);
  } else if(cmd == "STP"){
      clearDisplay();
      lcd.print("STOPPING?!?!?");
      stopMoving();
  } else {
      clearDisplay();
      lcd.print("WAT?");
  }

}

void motorTest(){
	leftDrive(1, 50, 720);
	rightDrive(1, 50, 720);
	delay(2000);
	leftDrive(0, 50, 720);
	rightDrive(0, 50, 720);
}

void breakTime(){
  clearDisplay();
  lcd.print("taking a break...");
  delay(2000);
  clearDisplay();
}



/////////////////////////// MOVEMENT COMMANDS //////////////////////////

void
leftDrive(int dir, int spd, long deg){
	if(dir){
    if(deg < 0){
      nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, spd, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
      nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, spd, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
    } else {
      nxshield.bank_b.motorRunDegrees(SH_Motor_1, SH_Direction_Reverse, spd, deg, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
      nxshield.bank_b.motorRunDegrees(SH_Motor_2, SH_Direction_Forward, spd, deg, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
    }
	} else {
    if(deg < 0){
      nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, spd, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
      nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, spd, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
    } else {
      nxshield.bank_b.motorRunDegrees(SH_Motor_1, SH_Direction_Forward, spd, deg, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
      nxshield.bank_b.motorRunDegrees(SH_Motor_2, SH_Direction_Reverse, spd, deg, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
    }
	}
}

void
rightDrive(int dir, int spd, long deg){
	if(dir){
    if(deg < 0){
      mmx.runUnlimited(MMX_Motor_1, MMX_Direction_Forward, spd, MMX_Completion_Dont_Wait, MMX_Next_Action_Brake);
  		mmx.runUnlimited(MMX_Motor_2, MMX_Direction_Reverse, spd, MMX_Completion_Dont_Wait, MMX_Next_Action_Brake);
    } else {
      mmx.runDegrees(MMX_Motor_1, MMX_Direction_Forward, spd, deg, MMX_Completion_Dont_Wait, MMX_Next_Action_Brake);
      mmx.runDegrees(MMX_Motor_2, MMX_Direction_Reverse, spd, deg, MMX_Completion_Dont_Wait, MMX_Next_Action_Brake);
    }
	} else {
    if(deg < 0){
      mmx.runUnlimited(MMX_Motor_1, MMX_Direction_Reverse, spd, MMX_Completion_Dont_Wait, MMX_Next_Action_Brake);
      mmx.runUnlimited(MMX_Motor_2, MMX_Direction_Forward, spd, MMX_Completion_Dont_Wait, MMX_Next_Action_Brake);
    } else {
      mmx.runDegrees(MMX_Motor_1, MMX_Direction_Reverse, spd, deg, MMX_Completion_Dont_Wait, MMX_Next_Action_Brake);
      mmx.runDegrees(MMX_Motor_2, MMX_Direction_Forward, spd, deg, MMX_Completion_Dont_Wait, MMX_Next_Action_Brake);
    }
	}
}

void rotate(int dir, int spd, long deg){
  if(dir){ // CW
    leftDrive(true, spd, deg);
    rightDrive(false, spd, deg);
  } else { // CCW
    leftDrive(false, spd, deg);
    rightDrive(true, spd, deg);
  }
}

void
forward(int spd, long deg){
  leftDrive(true, spd, deg);
  rightDrive(true, spd, deg);
}

void reverse(int spd, long deg){
  leftDrive(false, spd, deg);
  rightDrive(false, spd, deg);
}

void
hitter(int spd, long deg, int dir){
   	if(dir){
  		nxshield.bank_a.motorRunDegrees(SH_Motor_Both, SH_Direction_Reverse, spd, deg, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
	} else {
  		nxshield.bank_a.motorRunDegrees(SH_Motor_Both, SH_Direction_Forward, spd, deg, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
	}
}

// Forces all motors to stop.
void
stopMoving(){
  //mmx.runSeconds(MMX_Motor_Both, MMX_Direction_Reverse, 0, 0, MMX_Completion_Dont_Wait, SH_Next_Action_Brake);
  //nxshield.bank_b.motorRunSeconds(SH_Motor_Both, SH_Direction_Reverse, 0, 0, SH_Completion_Dont_Wait, SH_Next_Action_Brake);

  mmx.runDegrees(MMX_Motor_1, MMX_Direction_Forward, 1, 0, MMX_Completion_Dont_Wait, MMX_Next_Action_Brake);
  mmx.runDegrees(MMX_Motor_2, MMX_Direction_Reverse, 1, 0, MMX_Completion_Dont_Wait, MMX_Next_Action_Brake);
  nxshield.bank_b.motorRunDegrees(SH_Motor_1, SH_Direction_Reverse, 1, 0, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
  nxshield.bank_b.motorRunDegrees(SH_Motor_2, SH_Direction_Forward, 1, 0, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
}

///////////////////////////// LCD COMMANDS //////////////////////////////
void clearDisplay() {
  lcd.write(0xFE);
  lcd.write(0x01);
}

void setLCDCursor(byte cursor_position){
 lcd.write(0xFE); // ready LCD for special command
 lcd.write(0x80); // ready LCD to recieve cursor potition
 lcd.write(cursor_position); // send cursor position
}


//////////////////////////// MISC //////////////////////////////////////

// Activates the noise maker
void beep() {
 digitalWrite(6, HIGH);
 delay(50);
 digitalWrite(6, LOW);
}

void coloryLights(){
  nxshield.ledSetRGB(8,0,0); // guessing that this is the red pin...
  delay(1000);
  nxshield.ledSetRGB(0,8,0);
  delay(1000);
  nxshield.ledSetRGB(0,0,8);
  delay(1000);
  nxshield.ledSetRGB(0,0,0);
  delay(1000);
}


float StrToFloat(String str){
  char carray[str.length() + 1]; //determine size of the array
  str.toCharArray(carray, sizeof(carray)); //put str into an array
  return atof(carray);
}
