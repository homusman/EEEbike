#include <Arduino.h>
#include <PID.cpp>
#include <serialWithStartEnd.cpp>


//hardware config
//NOTE: phase current sensors depreciated in version 
#define phaseCurrentEn  0 // 0: phase current sensors are NOT installed, 1: phase current sensors are installed
#define multiplxCh8 1 // 0: nothing is connected to ch8, 1: thermistor is, 2: Itot current sensor is
#define bmsCommEn 0 // 0: communications through serial not enabled with selected BMS, 1: comm through serial is enabled
  //EITHER Itot current sensor is present OR BMS can measure current and communciate through serial + Iboost sensor MUST be present for proper boost converter operation.

//pin mappings

  //inverter outputs and their corresponding PWM channels
  #define UH 18
  #define UL 5
  #define VH 2
  #define VL 23
  #define WH 4
  #define WL 0

  #define UHch 1
  #define ULch 2
  #define VHch 3
  #define VLch 4
  #define WHch 5
  #define WLch 6

  #define PWMfreq 30000
  #define PWMres 8
  #define PWMmaxval 255 //equal to 2^PWMres-1 (hand calculate since preprocessor was being weird)
  #define PWMbuckmaxval 245 //should be at most 2^PWMres-1, may be lower since boot strap cant operate at 100%
  #define PWMboostmaxval 178 //duty cycle at which power output is maximum based on adafruit calculator
  #define PWMregenmaxval 178 //duty cycle at which power output is maximum based on adafruit calculator
  //boost controller outputs and their corresponding PWM channels

  #define SR 26//boost converter sychronous rectification switch output
  #define SW 27//boost converter low switch output

  #define SRch 7
  #define SWch 8

 //motor hall inputs
  #define hallA 19 
  #define hallB 21
  #define hallC 22

  //serial communication with BMS
  #define bmsRX 16
  #define bmsTX 17

  //multiplexer input & control
  #define muxIn 34//mux input pin
  #define muxA 32//mux control A
  #define muxB 33//mux control B
  #define muxC 25//mux control A

  //SPI display
  #define dispDC 12
  #define dispSDA 13
  #define dispSCL 14
  #define dispCS 15

  //current sensors
  #define IUSense 39
  #define IVSense 35
  #define IBSense 35//boost converter output current


//ESC behaviour config
#define PBattMax 1000*10^3//(mW) 
#define PBoostMax 2000*10^3//(mW)refers to max output/input power of boost converter at worst case, and is limited by max SC current at lowest SC voltage 
#define VBattMax 48*10^6//(uV)
#define VscMax 16*10^6//(uV)
#define VOvercharge 50*10^3//(uV) exceed defined maximum by this much to add some hysterisis and prevent bouncing between SC and batt recharge due to self discharge
#define TransitionThres 2*10^6//(uV) how many volts below VscMax does regen power transition begin

#define TSinkMax 70*10^3//(mK) max temperature of ESC as measured on heatsink before thermal throttling

//sensor zero point calibrations (dimensionless if not stated) 
int brake1rest; //ADC output of brake1/brake2 sensor when brakes are not squeezed at all. Increase this value to add some hysterisis
int brake2rest;
int boostrest; 
int throttlerest;

int calVsc = 17000; //(uV/mV) estimates based on volatge dividers nominal:17000 calculated using cascaded resistor divider formula
int calVbatt = 32500; //(uV/mV) nominal: 32500
int Vscrest = 142; //(mV) readMillivolt output with no voltage applied
int Vbattrest = 142; //(mV) readMillivolt output with no voltage applied
int Itotrest = 1667; //(mV)
int Iboostrest = 1667; //(mV)
int IUphaserest; //(mV)
int IVphaserest; //(mV)

#define vDivRatio  2/3// volatge divider ratio = Ra/Ra+Rb where Ra is resistor connected to ground, Rb is conneced to Vin
#define ACS712gain 15151// (uA/mV) from acs712 30A sensitivity is 66mV/A
#define ISenseScalar 22727// (uA/mV) ISenseScalar = (1/vDivRatio)*ACS712gain and should be calculated as a float and truncated by hand with the provided values as preprocessor only does integer division and possibly introduces error

// Variable initialisation
  //User control SETPOINT
  int PTotCommand; //(mW) mapped from corresponding analog in values
  int PBoostCommand; //(mW)

  //Measurements
  int Vbatt=0; //(uV)
  int Vsc=0; //(uV)
  int Tsink=0; //(mK)
  int Tmotor=0; //(mK)
  int Itot=0; //(uA)
  int Ibatt=0; //(uA)
  int Iboost=0; //(uA)
  bool hallAstate;
  bool hallBstate;
  bool hallCstate;

//log phase currents if they are present
  #if phaseCurrentEn == 1
  float IUPhase; //(mA)
  float IVPhase; //(mA)
  #endif
  

  //user inputs (dimensionless, just the ADC output)
  int throttle; //measured throttle command (proportional to power provided by battery) 
  int boost; //measured boost command (proportional to power provided by SC)
  int brake1; //either brake will command regen braking
  int brake2;
  int brakeTot; //aggregate of brake1 and brake2 command
  

  //internal
  int PBattCommand; //(mW)
  int PRegenCommand; //(mW)
  int PRegenCommandMax; //(mW)
  int PRegenCommandRaw; //(mW)

  int DutyBoost;
  int DutyInverter;
  uint8_t state;
  int highPhaseCh;
  int lowPhaseCh;

  long unsigned int prevT1;
  long unsigned int prevT2;
  
  TaskHandle_t cpu0task;
  TaskHandle_t cpu1task;

  //PID object initialisation
  #define Kp1 0.01
  #define Ki1 0
  #define Kd1 0
  #define tau1 0

  #define Kp2 0.01
  #define Ki2 0
  #define Kd2 0
  #define tau2 0

//for packing and unpacking serial
/*send serial commands as follows
BYTE PURPOSE
0    command     
1    data most significant bits
2    data ...
3    data ...
4    data least signifcat bits
*/
class serialreceived{
  public:
    byte receivedBytes[5];
    int getdata(){
      return (u_int)(receivedBytes[4]|receivedBytes[3]<<8|receivedBytes[2]<<16|receivedBytes[1]<<24);
    }
    byte getcmd(){
      return (byte)receivedBytes[0];
    }
};

serialreceived serial1receive;

serialWithStartEnd debugSerial(&Serial,serial1receive.receivedBytes,sizeof(serial1receive.receivedBytes)/sizeof(serial1receive.receivedBytes[0]),0xfe,0xff);

PIDController inverterController(Kp1,Ki1,Kd1,tau1,-PWMregenmaxval,PWMbuckmaxval,-PWMregenmaxval,PWMbuckmaxval);
PIDController boostController(Kp2,Ki2,Kd2,tau2,-PWMboostmaxval,PWMbuckmaxval,-PWMboostmaxval,PWMbuckmaxval);

int clamp(int x,int min,int max){
  if(min<=x<=max){
    return x;
  }
  else if(x>max){
    return max;
  }
  else if(x<min){
    return min;
  }
}

/*interfaces with the hardware taking duty cycles as input and 
setting appropriate control PWM pins as output to operate each 
set of half bridges in boost or buck mode*/
void commutateBoost(){
if(DutyBoost>0){
  //operate in forwards boost mode
  ledcWrite(SWch,DutyBoost);
  ledcWrite(SRch,0);
}
else if(DutyBoost<0){
  //operate in reverse buck mode
  ledcWrite(SWch,0);
  ledcWrite(SRch,DutyBoost);
}
else{
  //switch off
  ledcWrite(SWch,0);
  ledcWrite(SRch,0);
}
}

void commutateInverter(){
  //same as boost converter commutation but need to determine high state
  hallAstate = digitalRead(hallA);
  hallBstate = digitalRead(hallB);
  hallCstate = digitalRead(hallC);

  if (hallAstate == 0 && hallBstate == 0 && hallCstate == 1){
    state = 0;
  }
  else if (hallAstate == 1 && hallBstate == 0 && hallCstate == 1){
    state = 1;
  }
  else if (hallAstate == 1 && hallBstate == 0 && hallCstate == 0){
    state = 2;
  }
  else if (hallAstate == 1 && hallBstate == 1 && hallCstate == 0){
    state = 3;
  }
  else if (hallAstate == 0 && hallBstate == 1 && hallCstate == 0){
    state = 4;
  }
  else if (hallAstate == 0 && hallBstate == 1 && hallCstate == 1){
    state = 5;
  }
  else{
    //Serial.println("unexpected hall combination");
    DutyInverter = 0;
  }

  if (state == 1){
    //UV
    //Serial.println("UV");//DEBUG
    highPhaseCh = UHch;
    lowPhaseCh = VLch;
  }
  else if (state == 2){
    //UW
    //Serial.println("UW");//DEBUG
    highPhaseCh = UHch;
    lowPhaseCh = WLch;
  }
  else if (state == 3){
    //VW
    //Serial.println("VW");//DEBUG
    highPhaseCh = VHch;
    lowPhaseCh = WLch;
  }
  else if (state == 4){
    //VU
    //Serial.println("VU");//DEBUG
    highPhaseCh = VHch;
    lowPhaseCh = ULch;
  }
  else if (state == 5){
    //WU
    //Serial.println("WU");//DEBUG
    highPhaseCh = WHch;
    lowPhaseCh = ULch;
  }
  else if (state == 0){
    //WV
    //Serial.println("WV");//DEBUG
    highPhaseCh = WHch;
    lowPhaseCh = VLch;
  }

if(DutyInverter>0){
  ledcWrite(UHch,0);
  ledcWrite(VHch,0);
  ledcWrite(WHch,0);
  ledcWrite(ULch,0);
  ledcWrite(VLch,0);
  ledcWrite(WLch,0);

  //operate in forwards buck mode
  ledcWrite(highPhaseCh,DutyInverter);
  ledcWrite(lowPhaseCh,PWMmaxval);
}
else if(DutyInverter<0){
  //operate in reverse boost mode
  ledcWrite(UHch,0);
  ledcWrite(VHch,0);
  ledcWrite(WHch,0);
    
  ledcWrite(ULch,DutyInverter);
  ledcWrite(VLch,DutyInverter);
  ledcWrite(WLch,DutyInverter);
}
else{
  //switch off
  ledcWrite(UHch,0);
  ledcWrite(VHch,0);
  ledcWrite(WHch,0);
  ledcWrite(ULch,0);
  ledcWrite(VLch,0);
  ledcWrite(WLch,0);
  }
}

/*takes throttle, boost, brake ADC vals as input and 
outputs the physical command values, eg power and hence current that the hardware can control.
For E-storage positive power for sourcing, for 3-phase converter positive is for motoring 
This specific logic for mappings between sensor and command is arbitrary and based on what i subjectively think will feel natural*/

void mapInputsToCommandVals(){ 
  /*in all cases, brake command specifies the total negative power (recharging) 
  that is going back into one of the electrical systemsand overwrites the throttles control over positive power (discharging). Supercap has priority, if it is full 
  then the battery will absorb the energy instead. The power that commanded to be sent/provided 
  by either SC or batts will always be clamped to +-max defined. The regen commanded should smoothly interpolate as VSC reaches its maximum value.
  Consequentially the SC can be precharged by activating the brakes whilst also using the throttle as long as SC is not already full. 
  Because Iboost < 0 based on brake regen command and Ibatt > 0 based on throttle command*/
   //for IBoost
  PBoostCommand = 0;//clamp(map(boost,0,4095,0,PBoostMax),0,PBoostMax); TODO TEMPORARY FOR TESTING
  //for IBatt
  PBattCommand = clamp(map(throttle,0,4095,0,PBattMax),0,PBattMax); 

  PTotCommand = PBoostCommand + PBattCommand;


  //compute aggregate brake command
  brakeTot = 0;//clamp(brake1 + brake2,0,4095); TODO TEMPORARY FOR TESTING
  
  //raw regen power is mapped from aggregate brake cmd
  //0 regen command corresponds to both brakes at rest, when either brake is fully squeezed (ie 4095) max power output is commanded (equal to PSC max)
  PRegenCommandRaw = map(brakeTot, 0,4095,0,-PBoostMax); 

  //max regen power to clamp to provides a continuous transition between PBoostMax and PBattMax
  if (Vsc>VscMax-TransitionThres){
    PRegenCommandMax = -((PBattMax-PBoostMax)/(VscMax-(VscMax-TransitionThres))*(Vsc-(VscMax-TransitionThres))+ PBoostMax);//using y-y0 = (y1-y0/x1-x0)(x-x0)
  }
  else{
    PRegenCommandMax = -PBoostMax;
  }
  PRegenCommand = clamp(PRegenCommandRaw,0,PRegenCommandMax);

//associate regen power to either supercap or batt or none depending on current SC SOC or batt
  
// !!only enforce regen mode if brakes are squeezed
  if (PRegenCommand > 0){
    if (Vsc <= VscMax + VOvercharge){
      PBoostCommand = PRegenCommand; //(negative boost command is recharge back into the SC)
      //note how PBatt is not forced = 0 as in below modes, so supercap can be precharged by squeezing brakes AND using throttle
      }
    else if (Vbatt <= VBattMax + VOvercharge){
      //PBatt is not directly controlled, instead Pbatt + Pboost = Ptot with Pboost = 0 and PBatt = PRegenCommand, in this mode PBoost is forced zero to prevent rapidly jumping between recharging batt and SC
      PBoostCommand = 0;
      PTotCommand = PRegenCommand;
    }
    else{
      //If all storage is full then no regen is commanded and both PBatt and Pboost are forced = 0 (note, only if brakes are squeezed, motoring is unaffected)
      PBoostCommand = 0;
      PTotCommand = 0;
    }
  }
}

/*takes command values and ESC state as input to produce the duty cycles as output*/
void updatePIDs(){
  //controllers calculate power from sensors via P=IV (and that 1mW = 10^9 uA*uV). Since set point and control variable need to be same unit
inverterController.Update(PTotCommand*10^9,Itot*(48000000) /*Vbatt*/,micros()-prevT1);
prevT1 = micros();
DutyInverter = map(PTotCommand,0,PBattMax,0,255);//inverterController.out; TEMP

boostController.Update(PBoostCommand*10^9,Iboost*Vbatt,micros()-prevT2); 
prevT2 = micros();
DutyBoost = boostController.out;
}

#define weight 1
#define weight1 10
//helper function for readState(), no decimal to binary conversion is done in the function to reduce overhead, refer to CD4051 datasheet for correspondence
void inline setMuxChannel(bool a,bool b,bool c){digitalWrite(muxA,a);digitalWrite(muxB,b);digitalWrite(muxC,c);delayMicroseconds(0);}
/*poll sensors which determine ESC state, eg BMS, boost/buck currents, temperatures. This includes user inputs*/
/*intended for relatively low speed polling for user interaction and control*/
void readState(){
//read multiplexer sensors
setMuxChannel(0,0,0);
brake1 = analogRead(muxIn)-brake1rest;

setMuxChannel(1,0,0);
brake2 = analogRead(muxIn)-brake2rest;

setMuxChannel(0,1,0);
throttle = analogRead(muxIn)-throttlerest;


setMuxChannel(1,1,0);
boost = analogRead(muxIn)-boostrest;


setMuxChannel(0,0,1);
Vbatt = Vbatt*weight1+weight*calVbatt*(analogReadMilliVolts(muxIn)-Vbattrest);
Vbatt = Vbatt/(weight1+weight);

setMuxChannel(1,0,1);
Vsc = Vsc*weight1+weight*calVsc*((analogReadMilliVolts(muxIn))-Vscrest);
Vsc = Vsc/(weight1+weight);

setMuxChannel(0,1,1);
Tsink = Tsink*weight1+weight*analogReadMilliVolts(muxIn); //using volatge divider eqns and curve fitted for thermistor R(Tsink) = . Rearange for Tsink: Vout = ()*(5*Rs)/(R(Tsink)+Rs) where
Tsink = Tsink/(weight1+weight);

setMuxChannel(1,1,1);
#if multiplxCh8 == 1
Tmotor = analogReadMilliVolts(muxIn);
#elif multiplxCh8 == 2
/* MOVED TO readCurrent 
Tmotor = 0;
Itot = Itot*weight1+weight*(analogReadMilliVolts(muxIn)-Itotrest)*ISenseScalar;
Itot = Itot/(weight1+weight);
*/
#endif 

//read non multiplexed sensors
Iboost = Iboost*weight1+weight*(analogReadMilliVolts(IBSense)-Iboostrest)*ISenseScalar;
Iboost = Iboost/(weight1+weight);
#if phaseCurrentEn == 1
IUPhase = analogReadMilliVolts(IUSense);
IVPhase = analogReadMilliVolts(IVSense);
#endif

//get sensor values from BMS 
#if bmsCommEn == 1
Ibatt = bmsGetIBatt();
#endif

//determine physical state values based on what sensors are avaliable
#if multiplxCh8 != 2
Itot = Ibatt + Iboost; //if Itot is not directly measured
#endif
}

/*poll ADC for boost/buck currents. Intended for high speed polling used in main control loop*/
void readCurrent(){
  /*U and V phase current sense depreciated, change in next rev PCB. 
  (it was just for curiosity's sake anyways and isnt critical for the intended block commutation)
  For now, use these free ADC ports to do high speed polling of boost current 
  (which was connected to MUX, and isnt fast enough to guarentee control loop 
  can prevent over current condition)
  */

//DEBUG NOTE that a jumper needs to be connected between mux CH8 and IUSense

Itot = Itot*weight1+weight*(analogReadMilliVolts(IUSense)-Itotrest)*ISenseScalar;
Itot = Itot/(weight1+weight);

Iboost = Iboost*weight1+weight*(analogReadMilliVolts(IBSense)-Iboostrest)*ISenseScalar;
Iboost = Iboost/(weight1+weight);


}
/*draw stats and GUI to screen*/
void updateDisplay(){

}

#define calibrationIterations 100

void calibrateInputs(){
  brake1rest = 0;
  brake2rest = 0;
  throttlerest = 0;
  boostrest = 0;
    for (size_t i = 0; i < calibrationIterations; i++)
  {
  setMuxChannel(0,0,0);
  brake1rest += analogRead(muxIn);

  setMuxChannel(1,0,0);
  brake2rest += analogRead(muxIn);

  setMuxChannel(0,1,0);
  throttlerest += analogRead(muxIn);

  setMuxChannel(1,1,0);
  boostrest += analogRead(muxIn);
  }
  brake1rest = brake1rest/calibrationIterations;
  brake2rest = brake2rest/calibrationIterations;
  throttlerest = throttlerest/calibrationIterations;
  boostrest = boostrest/calibrationIterations;
}

//arg1 calibration value in mV
void calibrateVbatt(int data){
  calVbatt = 0;
  setMuxChannel(0,0,1);
  for (size_t i = 0; i < calibrationIterations; i++)
  {
    calVbatt += (int)(data/analogRead(muxIn)); //take a running sum
  }
  calVbatt = calVbatt/calibrationIterations;
}

//arg1 calibration value in mV
void calibrateVsc(int data){
  calVsc = 0;
  setMuxChannel(1,0,1);
  for (size_t i = 0; i < calibrationIterations; i++)
  {
    calVsc += (int)(data/analogRead(muxIn)); //take a running sum
  }
  calVsc = calVsc/calibrationIterations;
  
}

//should only be called when current sensors known to be passing no current
 void calibrateIsense(){
  Iboostrest = 0;
  Itotrest = 0;
  IUphaserest = 0;
  IVphaserest = 0;

    for (size_t i = 0; i < calibrationIterations; i++)
  {
    Iboostrest += analogReadMilliVolts(IBSense);

    #if multiplxCh8 == 2
      setMuxChannel(1,1,1);
      Itotrest += analogReadMilliVolts(muxIn);
    #endif

    #if phaseCurrentEn == 1
      IUphaserest += analogReadMilliVolts(IUSense);
      IVphaserest += analogReadMilliVolts(IVSense);
    #endif

  }
  Iboostrest = Iboostrest/calibrationIterations;
  Itotrest = Itotrest/calibrationIterations;
  IUphaserest = IUphaserest/calibrationIterations;
  IVphaserest = IVphaserest/calibrationIterations;
 }

/*do something based on what the received serial byte is*/
void serialCommand(byte command, int data = 0){
  //TODO implemenet serial commands
  switch(command){
    case 1:
      //calibrate input sensors (make sure all of them are in rest position)
      Serial.println("Calibrating inputs");
      calibrateInputs();
      break;
    case 2:
      //calibrate vbatt sense (enter calibration value in mV)
      Serial.print("Calibrating Vbatt using given current voltage ");
      Serial.print((int)data,DEC);
      Serial.println("mv");
      calibrateVbatt(data);
      break;
    case 3:
      //calibrate vsc sense (enter calibration value in mV)
      Serial.print("Calibrating Vsc using given data ");
      Serial.print((int)data,DEC);
      Serial.println("mv");
      calibrateVsc(data);
      break;
    case 4:
    Serial.println("Calibrating Isense (assuming currently no current)");
    calibrateIsense();
      break;
}
}

/* 
serialBMS JBDBMS;
class serialBMS{
  
void init(){
  HardwareSerial BMSport = Serial2;
  BMSport.begin(115200);
}
void sendReadCommand(String commandCode){
  Serial2.print("DD"+"A5"+commandCode+"00"+  +"77" ); //TODO figure out how the checksum is computed
}
void readBasicInfo(){
sendReadCommand("03"); //0x03 for basic info read command

}
void writeMOSFETstate(bool chargeState, bool dischargeState){ //logic 1 for charging/discharging enable
  if (chargeState == 1&& dischargeState == 1){
    char data = "00";
  }
  else if(chargeState == 0&& dischargeState ==1){
    char data = "01";
  }
  else if(chargeState == 1&& dischargeState ==0){
    char data = "02";
  }
  else if(chargeState == 0&& dischargeState ==0){
    char data = "03";
  }
  else{
  }
  Serial2.print("DD"+"5A"+"E1"+"02"+""+data+  +"77" );//TODO figure out how the checksum is computed
}

};
 */


//DEBUG0
int freqcounter0 = 0; //only core 0 should write to freqcounter0, counts how many times loop has excecuted

void loop0(void * pvParameters){
while(true){
  //****************** loop body here ******************

  commutateBoost();

  //DutyInverter = 150; //DEBUG 
  //commutateInverter();

  //for excecution time analysis
  freqcounter0 +=1;


  //****************** loop body here ******************
}
}

//DEBUG1
int freqcounter1 = 0; //for counting core1's own running freq
int freqcounter0last = 0; 
/* for counting and reporting core0's frequency. 
freqcounter0 (read only for core1, read/write for core0) - freqcounter0last (read/write for core1) = number of loops of core0 in one loop of core1
 is enough info since core1 also records the micros() time between each loop. (pr*/
unsigned long microslast1 = 0;

void loop1(void * pvParameters) {
while(true){

  //****************** loop body here ******************

  readState();

  //mapInputsToCommandVals();

  //updatePIDs();

  //updateDisplay();

  

  //check for new serial data
  debugSerial.recvWithStartEndMarkers();
  if (debugSerial.querynewdata()==true){
    //Serial.println("should be sending now...");
    //Serial.println(serial1receive.getcmd());
    //Serial.println(serial1receive.getdata());
    serialCommand(serial1receive.getcmd(),serial1receive.getdata());
    debugSerial.finishread();
  }

  //for excecution time analysis
  //note the || term in the timing if condition to account for when micros loops back to zer0 (~70 mins)
  freqcounter1 += 1;
  if (((micros()-microslast1 >= 1000000) || (micros()-microslast1 < 0)) && true){
    microslast1=micros();

    Serial.print(" count0, ");
    Serial.println(freqcounter0-freqcounter0last);
    Serial.print(" count1, ");
    Serial.println(freqcounter1);

    freqcounter0last=freqcounter0;
    freqcounter1=0;
  }

  //for debugging PID
    if (false){

    }

  //for debugging sensor values
    if (((micros()-microslast1 >= 100000) || (micros()-microslast1 < 0)) && false){
    microslast1=micros();
    //Serial.print("us ");
    //Serial.print(tlast-micros());
    
    Serial.print(", Iboost(mA) ");
    Serial.print(Iboost/1000);
    Serial.print(", Itot(mA) ");
    Serial.print(Itot/1000);
    Serial.print(", DutyBoost ");
    Serial.print(DutyBoost);
    Serial.print(", DutyInverter ");
    Serial.print(DutyInverter);
    Serial.print(", Pboostcmd(mW) ");
    Serial.print(PBoostCommand);
    Serial.print(", Ptotcmd(mW) ");
    Serial.print(PTotCommand);
    Serial.print(", throttle ADC ");
    Serial.print(throttle);
    Serial.print(", boost ");
    Serial.print(boost);
    //Serial.print(", PReg(mW) ");
    //Serial.print(PRegenCommand);
    Serial.print(", Vbatt(mV) ");
    Serial.print(Vbatt/1000);
    Serial.print(", Vsc(mV) ");
    Serial.println(Vsc/1000);

    /*
    setMuxChannel(1,0,1);
    Serial.println(analogReadMilliVolts(muxIn));

    setMuxChannel(1,1,1);
    Serial.println(analogReadMilliVolts(muxIn));

    Serial.println(analogReadMilliVolts(IBSense));
    */
  }


  //****************** loop body here ******************
}
}

void setup() {
  debugSerial.begin(115200);

  //JBDBMS.init();

  //init loops on core 0 and core 1
  xTaskCreatePinnedToCore(
      loop0, /* Function to implement the task */
      "fast loop", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &cpu0task,  /* Task handle. */
      0); /* Core where the task should run */
      
  xTaskCreatePinnedToCore(
      loop1, /* Function to implement the task */
      "slow loop", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &cpu1task,  /* Task handle. */
      0); /* Core where the task should run */

//set pinmodes
pinMode(muxIn,INPUT);
pinMode(muxA,OUTPUT);
pinMode(muxB,OUTPUT);
pinMode(muxC,OUTPUT);

pinMode(hallA,INPUT_PULLUP);
pinMode(hallB,INPUT_PULLUP);
pinMode(hallC,INPUT_PULLUP);

pinMode(IUSense,INPUT);
pinMode(IVSense,INPUT);

pinMode(IBSense,INPUT);

pinMode(muxA,OUTPUT);
pinMode(muxB,OUTPUT);
pinMode(muxC,OUTPUT);

pinMode(UH,OUTPUT);
pinMode(UL,OUTPUT);
pinMode(VH,OUTPUT);
pinMode(VL,OUTPUT);
pinMode(WH,OUTPUT);
pinMode(WL,OUTPUT);

pinMode(SR,OUTPUT);
pinMode(SW,OUTPUT);

#if bmsCommEn==1
pinMode(bmsRX,INPUT);
pinMode(bmsTX,OUTPUT);
#else
pinMode(bmsRX,INPUT);
pinMode(bmsTX,INPUT);
#endif

pinMode(dispDC,OUTPUT);
pinMode(dispSDA,OUTPUT);
pinMode(dispSCL,OUTPUT);
pinMode(dispCS,OUTPUT);

//init PWM
ledcSetup(UHch,PWMfreq,PWMres);
ledcAttachPin(UH,UHch);

ledcSetup(ULch,PWMfreq,PWMres);
ledcAttachPin(UL,ULch);

ledcSetup(VHch,PWMfreq,PWMres);
ledcAttachPin(VH,VHch);

ledcSetup(VLch,PWMfreq,PWMres);
ledcAttachPin(VL,VLch);

ledcSetup(WHch,PWMfreq,PWMres);
ledcAttachPin(WH,WHch);

ledcSetup(WLch,PWMfreq,PWMres);
ledcAttachPin(WL,WLch);

ledcSetup(SWch,PWMfreq,PWMres);
ledcAttachPin(SW,SWch);

ledcSetup(SRch,PWMfreq,PWMres);
ledcAttachPin(SR,SRch);

prevT1 = prevT2 = micros();

//calibrate sensors
delay(100);
calibrateInputs();
calibrateIsense();

//exceptions
  if (bmsCommEn == 0 && multiplxCh8 != 2){
    Serial.println("Warning, neither I_batt or I_tot being sensed");
  }
Serial.println("Setup complete");
}

void loop() {
 
  Serial.println("Starting main loop...");
  while (true){}
 
}