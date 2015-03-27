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
#include <MagicWand.h>
#include <MsTimer2.h>
#include <NumericPad.h>
#include <NXShield.h>
#include <NXShieldAGS.h>
#include <NXShieldI2C.h>
#include <NXTCam.h>
#include <NXTCurrentMeter.h>
#include <NXTHID.h>
#include <NXTLight.h>
#include <NXTPowerMeter.h>
#include <NXTServo.h>
#include <NXTTouch.h>
#include <NXTUS.h>
#include <NXTVoltMeter.h>
#include <PFMate.h>
#include <PiLight.h>
#include <PSPNx.h>
#include <RCXLight.h>
#include <RTC.h>
#include <SHDefines.h>
#include <SoftI2cMaster.h>
#include <SumoEyes.h>
#include <Wire.h>
#include <NXShield.h>
#include <NXTUS.h>

#include<NXTMMX.h>
//#include<NXTI2CDevice.h>

#include <SoftwareSerial.h>

// The shield
NXShield nxshield();

// 2 line LCD on arduino digital pin #6
SoftwareSerial lcd(2, 6);

//
// Declare the i2c devices used on NXShield(s).
//
NXTUS       sonarLeft; // left
NXTUS       sonarRight; // right
NXTUS       sonarBack;
NXTMMX      mmx; // Multiplexer for chain morors

void setup(){
  Serial.begin(115200);
  
  lcd.begin(9600); 
  lcd.print("Setup Starting...");
  delay(500);
  
  nxshield.init(SH_HardwareI2C);
  nxshield.bank_a.setAddress(0x07);
  
  // Check battery voltage on startup. Warn if low.
  /*
  float batVolt = (float) mmx.getBatteryVoltage() / 1000;
  if(batVolt < 7.50) {
    clearDisplay(); 
    //beep();
    lcd.print("LOW VOLTAGE!!!");
    setLCDCursor(16);
    lcd.print("voltage: ");
    lcd.print(batVolt);
  } else {
    clearDisplay();
    lcd.print("Press GO!");
    setLCDCursor(16);
    lcd.print("voltage: ");
    lcd.print(batVolt); 
  }
  */
  
  nxshield.waitForButtonPress(BTN_GO);
  
  nxshield.bank_a.motorReset();
  nxshield.bank_b.motorReset();
  
 // nxshield.bank_a.setAddress(0x07);

} // end setup

void loop() {

  delay(2000);
  elevatorRaise(50, 2);
  delay(2000);
  elevatorLower(50, 2);
 
  
  //lcd.print("Operation Complete?");
 // delay(5000);
}

///////////////////////////// ELEVATOR COMMANDS ////////////////////////////

void elevatorRaise(int speed1, long rotations) {
  mmx.runRotations(MMX_Motor_1, MMX_Direction_Reverse, speed1, rotations, MMX_Completion_Dont_Wait, MMX_Next_Action_Float);
  mmx.runRotations(MMX_Motor_2, MMX_Direction_Forward, speed1, rotations, MMX_Completion_Dont_Wait, MMX_Next_Action_Float); 
}

void elevatorLower(int speed1, long rotations) {
  mmx.runRotations(MMX_Motor_1, MMX_Direction_Forward, speed1, rotations, MMX_Completion_Dont_Wait, MMX_Next_Action_Float);
  mmx.runRotations(MMX_Motor_2, MMX_Direction_Reverse, speed1, rotations, MMX_Completion_Dont_Wait, MMX_Next_Action_Float); 
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

/////////////////////////// MOVEMENT COMMANDS //////////////////////////

void
moveForward(int speed1){
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, speed1);
}

void
moveBackward(int speed1){
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, speed1);
}

void
stopMoving(){
//  nxshield.bank_a.motorStop(SH_Motor_Both, SH_Next_Action_Float);
//  nxshield.bank_b.motorStop(SH_Motor_Both, SH_Next_Action_Float);
  nxshield.bank_a.motorRunSeconds(SH_Motor_Both, SH_Direction_Reverse, 0, 0, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
  nxshield.bank_b.motorRunSeconds(SH_Motor_Both, SH_Direction_Reverse, 0, 0, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
}
void
strafeRight(int speed1){
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, speed1);
}

void
strafeLeft(int speed1){
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, speed1);
  
}

void
turnClockwise(int speed1){
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, speed1);
}
void
turnCounterClockwise(int speed1){
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, speed1);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, speed1);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, speed1);
}

void
diagonal(){
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, 40);
  nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, 90);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, 90);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, 40);
  
}


