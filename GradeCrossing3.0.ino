// GradeCrossing controller
//
// A Nano-based sketch to control one or more grade crossings
// -- primarily for the flashing lights
// -- with optional bell and gate controls
// -- gate may be digital or servo controlled
// -- Requires minimum of 3 track sensors - east/west/center
//
// *************************************************************************
// Revision history:
#define Version "GradeCrossing 3.0 2022/07/10 10:00"
//   2022/07/08 J. Schmidt 3.0 Update gate/servo processing with EEProm
//                             Simplify exit processing
//   2022/04/30 J. Schmidt 2.0 Update gate & servo processing
//                             Optionally use VarSpeedServo
//                             fixed TimeOutSecs starts after WaitABitSecs
//   2022/02/20 J. Schmidt 1.3 Redo sensor sensitivity
//   2022/02/02 J. Schmidt 1.2a Fix GateDly on xing cleared
//   2022/01/25 J. Schmidt 1.2 Renumber types, add NoGate & code for multiple gates; 
//   2022/01/22 J. Schmidt 1.1 Fix start flash; fix refresh of sensors;
//   2022/01/13 J. Schmidt 1.0 Fix WaitABitSecs foNr Xidx crossings
//   2022/01/08 J. Schmidt 1.0 Add the WaitABitSecs to delay signal after distant hit
// *************************************************************************
// This code is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed WITHOUT ANY WARRANTY; 
// without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// http://www.gnu.org/licenses/.
// *************************************************************************
//
#define VSServo true
#if VSServo
  #include <VarSpeedServo.h>
  // speed 10 is slow-ish
  #define ServoSpeed 10
#else
  #include <Servo.h>
#endif
#include "EEPROM-Storage.h"
// *************************************************************************
// Timing definitions
// |-WaitABitSecs-|B4 Start signal
//                |GateDlyMs|Delay for gate drop
//                |--TimeoutSecs---|Clear signal if crossing not hit
//                                      |GateDlyMs|Delay for gate raise
//                                                |LiteDlyMs|Delay sig stop
//                                      |---TimeoutSecs---|Cleanup
// ES.................................CS.................................WS
// East sensors                 Crossing sensors               West sensors
//
// WaitABitSecs - delay from distant sensor hit to signal starts
// GateDlyMS - delay between lites start and gates drop, gates raise and lites stop
// TimeOutSecs - timeout if distant sensor hit but train stops short of crossing
//             - also on exit timeout for exit sensor ignored
// *************************************************************************
// *** Global definitions - dont change
// Sensor types for in-track sensors: digital or analog
#define AlgType   1
#define DigType   2
// Crossing gate type:
#define NoGate    0
// DigType for on/off control
// SrvoType for servo-controlled gates
#define SrvoType  3
//
// *************************************************************************
// USER TUNEABLE Defines
// *************************************************************************
// Define the active crossings - NumXings crossings
// each crossing - up to NumTrks tracks, up to 3 sensors per track
// number of gate controls per crossing
#define NumXings 1
#define NumTrks  2
// don't change NumGates
#define NumGates 2
// dont change:
#define NumSnsrs NumTrks * 3
// for above: east sensors[NumTrks], west sensors[NumTrks], crossing sensors[NumTrks]
//
// Define crossing (XingDef) array elements
typedef struct {
  // input sensors analog or digital - DigType - AlgType
  int SnsrType; 
  // sensors (EastSnsr[NumTrks], WestSnsr[NumTrks], XingSnsr[NumTrks])
  uint8_t SnsrLst[NumSnsrs];
  // first crossing lamp address of two in sequence - left/right lamp flash control
  uint8_t FirstLite;
  // sound address - digital mode or empty 0
  uint8_t SoundCntl;
  // gate address - digital mode or empty 0
  uint8_t GateCntl[NumGates];
  // gate potentiometers - servo mode or empty 0
  uint8_t GatePots[NumGates*2];
  // GateType SrvoType or DigType
  int GateType; 
} XingDef;
//
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// DEFINITIONS for Central Vermont South Coventry
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// main+siding south, 1 trk at crossing, 1 trk north
// Define the crossing by the following array
XingDef Crossing [NumXings] =
//  SnsrType, EastSnsr[NumTrks], WestSnsr[NumTrks], XingSnsr[NumTrks], 
  {  AlgType, A3,  0,            A0, A2,            A1,    0,           
//     FirstLite, SoundCntl, GateCntl, 
       2,         4,         9, 10,    
//     GatePots,    
//     Up,Dwn,Up,Dwn     GateType
       A6,A7, A6,A7,     SrvoType};
// and the WaitABitSecs array
// East wait, West wait
int WaitABitSecs[NumXings][NumTrks*2] =
  {0,0,0,0};
//
// Sensitivity - sensor must be within % of lowest to trigger
#define Sentivity 10
// if digital track sensors use clear for occupied, true
#define DigInvert true 
//
// FlashIntvl - ms per flash for crossing lites
#define FlashIntvl 700 
// TimeoutSecs - turn off after distant hit but center not hit
#define TimeoutSecs 20
// GateDlyMs - number of ms after sensing and lites/bell start before gates drop
//           - also number of ms after sensing clears gates go up
#define GateDlyMs 4500 
// LiteDlyMs - number of ms after sensing clears that lites and bell stop
#define LiteDlyMs 5000 
// END DEFINITIONS for Central Vermont South Coventry
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
/*
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// DEFINITIONS for Central Vermont Stafford
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Define the active crossings - NumXings crossings
// each crossing - up to NumTrks tracks, up to 3 sensors per track
// number of gate controls per crossing
#define NumXings 1
#define NumTrks  1
// 1 trk south, 2 trks at crossing, 1 trk north
// delay approach from south 10 secs
// A crossing is defined by the following array
XingDef Crossing [NumXings] =
//  SnsrType, EastSnsr[NumTrks], WestSnsr[NumTrks], XingSnsr[NumTrks], 
  {  AlgType, A0, 0,             A3,    0,          A1,   A2,               
//     FirstLite, SoundCntl, GateCntl, GateType
       2,         4,         0, 0,     NoGate};
// and the WaitABitSecs array
// East wait, West wait
int WaitABitSecs[NumXings][NumTrks*2] =
  {10,  0,  0,  0};
//
// Sensitivity - sensor must be within % of lowest to trigger
#define Sentivity 2
// if digital track sensors use clear for occupied, true
#define DigInvert true 
//
// FlashIntvl - ms per flash for crossing lites
#define FlashIntvl 700 
// TimeoutSecs - turn off after distant hit but center not hit
#define TimeoutSecs 20
// GateDlyMs - number of ms after sensing and lites/bell start before gates drop
//           - also number of ms after sensing clears gates go up
#define GateDlyMs 4500 
// LiteDlyMs - number of ms after sensing clears that lites and bell stop
#define LiteDlyMs 3000 
// END DEFINITIONS for Central Vermont Stafford
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// */
/*
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// DEFINITIONS for Central Vermont Monson
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Define the active crossings - NumXings crossings
// each crossing - up to NumTrks tracks, up to 3 sensors per track
// number of gate controls per crossing
#define NumXings 1
#define NumTrks  1
// 1 trk south, 2 trks at crossing, 1 trk north
// delay approach from south 0 secs
// approach from south probably exits on trk 2 main north
// A crossing is defined by the following array
XingDef Crossing [NumXings] =
//  SnsrType, EastSnsr[NumTrks], WestSnsr[NumTrks], XingSnsr[NumTrks], 
  {  AlgType,    A0, 0,            A1, 0,            A2,  A3,     
//     FirstLite, SoundCntl, GateCntl, GateType
       2,         4,         0, 0,     NoGate};
// and the WaitABitSecs array
// East wait, West wait
int WaitABitSecs[NumXings][NumTrks*2] =
  {0,  0,  0,  0};
// Sensitivity - sensor must be within % of lowest to trigger
#define Sentivity 3
// if digital track sensors use clear for occupied, true
#define DigInvert true 
//
// FlashIntvl - ms per flash for crossing lites
#define FlashIntvl 700 
// TimeoutSecs - turn off after distant hit but center not hit
#define TimeoutSecs 20
// GateDlyMs - number of ms after sensing and lites/bell start before gates drop
//           - also number of ms after sensing clears gates go up
#define GateDlyMs 4500 
// LiteDlyMs - number of ms after sensing clears that lites and bell stop
#define LiteDlyMs 3000 
// END DEFINITIONS for Central Vermont Monson
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// */
// *************************************************************************
// END USER TUNEABLE Defines
// *************************************************************************
// Change anything below here at your own risk
// *************************************************************************
// analog sampling controls - needed to average light flicker
// length of sample integration in millisecs
// 34 ms for 60 cycle lighting mains
// 40 ms for 50 cycle lighting mains
#define LoopMS 34
// time to hold occupied status minimize false clears
#define HoldOccMS 1000
// Servo motion mapping
#define FullArc   160
#if VSServo
  VarSpeedServo CrossingGate[NumXings][NumGates];
#else
  Servo CrossingGate[NumXings][NumGates];
#endif
// high/low defines for digital gate
#define GateDown LOW // HIGH
#define GateUp   HIGH // LOW
// high/low defines for sound
#define SndOff   HIGH
#define SndOn    LOW
//
// define sensor groups
#define East  0
#define West  NumTrks
#define Cntr  NumTrks * 2
//
// define lampon/off
#define LampOn   LOW
#define LampOff  HIGH
//
// define setup flash modes
#define FlashOn    0 
#define FlashOff   1
#define FlashAlt   2
//
// Local Servo defines for gate
int LGateFlag;
int LGateUD   [NumGates][2];
// Defines for EEPROM for Servo processing
// addresses in EEPROM
#define E_Flag 0
#define E_GateDown0 E_Flag      + 2 + 1
#define E_GateDown1 E_GateDown0 + 2 + 1
#define E_GateUp0   E_GateDown1 + 2 + 1
#define E_GateUp1   E_GateUp0   + 2 + 1
#define E_Okay   2
#define E_Chngd  3
#define MinPotMove 10
EEPROMStorage<uint16_t> E_FlagV (E_Flag, 0); // EEPROM Flag
EEPROMStorage<uint16_t> E_DownV0(E_GateDown0, 0);  // EEPROM E_GateDown0 
EEPROMStorage<uint16_t> E_DownV1(E_GateDown1, 0);  // EEPROM E_GateDown1 
EEPROMStorage<uint16_t> E_UpV0  (E_GateUp0, 0);    // EEPROM E_GateUp0 
EEPROMStorage<uint16_t> E_UpV1  (E_GateUp1, 0);    // EEPROM E_GateUp1 
// 
// analog sensors are recalibrated frequently 

//    to account for changes in lighting
//  - such as sunlight moving through a window
// minutes between recalibrate low/high analog sensor values
#define RecalibrateMts 5
//
// ****************** Trace & Debug
// 0 - no trace; 1 - events; 2 - flow and data
//   3 - even more
#define Trace 0
// delay in ms for main loop sampling 
// -- if tracing/debugging
#define MainLoopDlyMS 2000
// if SnsrActiveLED is HOME-zero, light the 
//     LED indicated if any sensor is active
#define SnsrActiveLED 13
//#define SnsrActiveLED 0
#if Trace > 0
  String EWest[3]   = {"East","West","Cntr"};
  String SenStat[4] = {"Clear","Occupied","Ignore","Clearing"};
#endif 
//
// ******************* Variable definitions
boolean Occpied[NumSnsrs]; // current pass status
typedef struct {
  // data arrays for sensors
  int Smallest   [NumSnsrs]; // smallest analog values seen
  int Largest    [NumSnsrs]; // largest analog values seen
  int NowOccStat [NumSnsrs]; // current occupied state
  boolean Chngd  [NumSnsrs]; // state changed
  unsigned long 
       HoldOccTM [NumSnsrs]; // timing to eliminate false clears
} XingDatStr;
//
XingDatStr XingDat [NumXings];
//
// NowOccStat values
// - sensor is not occupied
#define StatClr   0
// - sensor is occupied
#define StatOcc   1
// - ignore sensor when exiting train covers
#define StatIgn   2
// train hit ignore sensor - clear when clear
#define StatClrng 3
//
// Crossing timing phase array & definitions
unsigned long TimeoutTM  [NumXings]; // time for dead move timeout
boolean       XingOcc    [NumXings]; // is the crossing occupied
unsigned long LstOccRstTM[NumXings]; // timing to clear ign/clrng
int           XingPhase  [NumXings]; // next phase
unsigned long NxtPhaseTM [NumXings]; // time for next phase processing
unsigned long NxtFlashTM [NumXings]; // time for next lite flash
boolean       FlashCycle [NumXings]; // cycle for next lite flash
// Phases
// no activity
#define XingIdle      0 
// start lites and bell 
#define XingStrtBell  1
// drop the gate
#define XingLowerGate  2 
// raise the gate
#define XingRaiseGate 3 
// stop lites & bell & clean up
#define XingCleanup   4
//
// LOOP variables
int           Xidx; // index for Crossings
int           Sidx; // index for sensors within crossings
int           Lidx; // local index
unsigned long CycleStrtMs; // cycle start current time
unsigned long Secs;        // current time in seconds
unsigned long Mts;         // current time in minutes
//
unsigned long SnsrActiveMS;// expire time for SnsrActiveLED
unsigned long Cycles;      // count of loops
unsigned long CycleMs;     // total ms in cycle
unsigned long ThisCycleMs; // total ms in this cycle
unsigned long ResetStrtMts;// last analog reset time in minutes
//
// ============= ReadSensors ======================
// read the sensors and set Occpied
// as well as Largest/Smallest if analog
void ReadSensors (){
// ============= ReadSensors ======================
for (Sidx = 0; Sidx < NumSnsrs; ++Sidx)
    {XingDat[Xidx].Chngd[Sidx] = false;} // for Sidx
    
if (Crossing[Xidx].SnsrType == DigType){
// read digital sensors
for (Sidx = 0; Sidx < NumSnsrs; ++Sidx){
      if (Crossing[Xidx].SnsrLst[Sidx] > 0){
      Occpied[Sidx] = digitalRead(Crossing[Xidx].SnsrLst[Sidx]);
      if (DigInvert){
       Occpied[Sidx] == true?false:true;
       }// invert
      } // if Crossing[Xidx].SnsrLst[Sidx]> 0
} // for Sidx
} // process digital sensors
else { 
// read analog sensors
  long int Smpls, Tint;
  long int Vals[NumSnsrs];
  unsigned long CycleStrt;
  for (Sidx = 0; Sidx < NumSnsrs; ++Sidx){
     Vals   [Sidx] = 0;
     Occpied[Sidx] = false;}
  Smpls = 0;
  CycleStrt = millis();
  do { // do the integration over LoopMS time
    // read all sensors
    for (Sidx = 0; Sidx < NumSnsrs; ++Sidx)
      {if (Crossing[Xidx].SnsrLst[Sidx] > 0)
        {Vals[Sidx] += analogRead(Crossing[Xidx].SnsrLst[Sidx]);}
	  }// for-if
    ++Smpls;
  } while (CycleStrt + LoopMS > millis());
  for (Sidx = 0; Sidx < NumSnsrs; ++Sidx)
    {if (Crossing[Xidx].SnsrLst[Sidx] > 0){
      // active sensor
    Vals[Sidx] /= Smpls; // get average
	// calc min/max
    XingDat[Xidx].Largest[Sidx] = max(XingDat[Xidx].Largest[Sidx], Vals[Sidx]);
    if (XingDat[Xidx].Smallest[Sidx] == 9999)
                    {XingDat[Xidx].Smallest[Sidx] = Vals[Sidx]*.8;}
    XingDat[Xidx].Smallest[Sidx] = min(XingDat[Xidx].Smallest[Sidx], Vals[Sidx]);
    #if Trace > 2
      Serial.print  ("Read ");
      Serial.print  (Sidx);
      Serial.print  (" sensor ");
      Serial.print  (Crossing[Xidx].SnsrLst[Sidx]);
      Serial.print  (" hi ");
      Serial.print  (XingDat[Xidx].Largest [Sidx]);
      Serial.print  (" lo ");
      Serial.print  (XingDat[Xidx].Smallest[Sidx]);
      Serial.print  (" value ");
      Serial.println(Vals[Sidx]);
      #endif  
    Tint = XingDat[Xidx].Largest[Sidx]-XingDat[Xidx].Smallest[Sidx];
    if (Tint > XingDat[Xidx].Largest[Sidx]*.15){// check range is meaningful
      Tint = max((Tint*Sentivity)/100,1); // calculate minimum drop
      if (Vals[Sidx] <= XingDat[Xidx].Smallest[Sidx]+Tint) // if in the lower limit
        {Occpied[Sidx] = true;
    #if Trace > 0
      Serial.print  ("Now occupied ");
      Serial.print  (Sidx);
      Serial.print  (" sensor ");
      Serial.print  (Crossing[Xidx].SnsrLst[Sidx]);
      Serial.print  (" hi ");
      Serial.print  (XingDat[Xidx].Largest[Sidx]);
      Serial.print  (" lo ");
      Serial.print  (XingDat[Xidx].Smallest[Sidx]);
      Serial.print  (" sentvy ");
      Serial.print  (Tint);
      Serial.print  (" lo+sens ");
      Serial.print  (XingDat[Xidx].Smallest[Sidx]+Tint);
      Serial.print  (" >value ");
      Serial.println(Vals[Sidx]);
      #endif
      }// in lower limit 
    } // if Tint
    } // if active sensor
    } // for Sidx
}// read analog sensors
//
// process sensors
//
  #if Trace > 2
    Serial.println("sensor/occ/changed/status ");
    Serial.print  ("RSb4:");
    for (Sidx = 0; Sidx < NumSnsrs; ++Sidx){
    if (Crossing[Xidx].SnsrLst[Sidx] > 0) {//sensor exists
    Serial.print  (SnsrNm(Sidx));
    Serial.print  ("/");
    Serial.print  (Occpied[Sidx]);
    Serial.print  ("/");
    Serial.print  (XingDat[Xidx].Chngd[Sidx]);
    Serial.print  ("/");
    Serial.print  (SenStat[XingDat[Xidx].NowOccStat[Sidx]]);
    Serial.print  (" ");}
    } // for Sidx
    Serial.println  (" ");
    #endif
for (Sidx = 0; Sidx < NumSnsrs; ++Sidx){
 if (Crossing[Xidx].SnsrLst[Sidx] > 0) {//sensor exists
 if (Occpied[Sidx] == true){
  XingDat[Xidx].HoldOccTM[Sidx] = millis() + HoldOccMS;
  switch (XingDat[Xidx].NowOccStat[Sidx]){
    case StatClr:
      // was clear, set occupied 
      XingDat[Xidx].NowOccStat[Sidx] =  StatOcc;
      XingDat[Xidx].Chngd     [Sidx] =  true;
      #if Trace > 0
        RptSnsr (Sidx);
        #endif
	    if (Sidx < Cntr) // set opposite ignore if approach
	     {SetPartners (Sidx<West?West:East, StatIgn);}
      // was clear, set occupied
	  break;
    case StatIgn:
      // was ignore, set clearing
      XingDat[Xidx].Chngd[Sidx] =  true;
	    XingDat[Xidx].NowOccStat[Sidx] =  StatClrng;
      // was clear, set occupied
	  break;
    } // switch
  } // if Occpied
  //
  else{// Occpied is false
  if (XingDat[Xidx].HoldOccTM[Sidx] < millis()){
    // holdocc has expired
  switch (XingDat[Xidx].NowOccStat[Sidx]){
    case StatOcc:
      // was occupied set clear
	    XingDat[Xidx].NowOccStat[Sidx] = StatClr;
      XingDat[Xidx].Chngd[Sidx] =  true;
	    break;
    case StatClrng:
      // was clearing set partners clear
	    SetPartners ((Sidx/NumTrks)*NumTrks, StatClr);
      XingDat[Xidx].Chngd[Sidx] =  true;
	    break;
    } // switch
  }// if hold time expired
  }// Occpied is false
 } // sensor exists
//
if (SnsrActiveLED > 0 && Occpied[Sidx] == true)
  {digitalWrite(SnsrActiveLED, HIGH);
   SnsrActiveMS = millis();
   } // SnsrActiveLED
} // for Sidx
  #if Trace > 0
    Serial.println("sensor/occ/changed/status ");
    Serial.print  ("RSaf:");
    for (Sidx = 0; Sidx < NumSnsrs; ++Sidx){
    if (Crossing[Xidx].SnsrLst[Sidx] > 0) {//sensor exists
    Serial.print  (SnsrNm(Sidx));
    Serial.print  ("/");
    Serial.print  (Occpied[Sidx]);
    Serial.print  ("/");
    Serial.print  (XingDat[Xidx].Chngd[Sidx]);
    Serial.print  ("/");
    Serial.print  (SenStat[XingDat[Xidx].NowOccStat[Sidx]]);
    Serial.print  (" ");}
    } // for Sidx
    Serial.println  (" ");
    #endif
}// ReadSensors
//
// ============= ProcessSensors ======================
void ProcessSensors (){
// ============= ProcessSensors ======================
// process deadman timeout even if occupied
  if (TimeoutTM[Xidx] > 0 && TimeoutTM[Xidx] < millis()){// active & expired
      #if Trace > 0
        Serial.println("Process deadman timeout");
        #endif
      TimeoutTM [Xidx] = 0;
      XingOcc   [Xidx] = false;
      XingPhase [Xidx] = XingRaiseGate;
      NxtPhaseTM[Xidx] = millis();
	    LstOccRstTM[Xidx] = 0;
      for (Sidx = 0; Sidx < NumSnsrs; ++Sidx){
        // check/clear the ignore flag
        if (XingDat[Xidx].NowOccStat[Sidx] != StatOcc)
           {XingDat[Xidx].NowOccStat[Sidx] = StatClr;
	          #if Trace > 0
             RptSnsr(Sidx);
             #endif
           }
      } // for Sidx
      }// invoke deadman & clear
//
// check endpoints changed
  for (Sidx = 0; Sidx < Cntr; ++Sidx){
    if (XingDat[Xidx].Chngd[Sidx] == true){
    switch (XingDat[Xidx].NowOccStat[Sidx]){
	  case StatOcc:
      // endpoint became occupied
      // set timeout and start lites/bell
      #if Trace > 0
        Serial.print  ("Endpoint now live set deadman ");
        Serial.println(Sidx);
        #endif
      TimeoutTM [Xidx] = millis() + (TimeoutSecs + WaitABitSecs[Xidx][Sidx]) * 1000;
      if (XingOcc[Xidx] == false){
        XingPhase [Xidx] = XingStrtBell;
        NxtPhaseTM[Xidx] = millis() + WaitABitSecs[Xidx][Sidx] * 1000;
	    XingOcc   [Xidx] = true;}
	    break;
	  case StatClrng:
	    XingOcc[Xidx] = true;
	    break; 
	  case StatIgn:
      // endpoint became occupied while in ignore state
	    XingDat[Xidx].NowOccStat[Sidx] = StatClrng;
	    break; 
      }// switch now occupied
    }// Changed   
  }// for endpoints Sidx
  //
  // check center changed
  for (Sidx = Cntr; Sidx < Cntr+NumTrks; ++Sidx){
    if (XingDat[Xidx].Chngd[Sidx] == true){
      if (XingDat[Xidx].NowOccStat[Sidx] == StatOcc){
// center became occupied
      #if Trace > 0
        Serial.print  ("Centerpoint now occupied clear deadman ");
        Serial.println(Sidx);
        #endif
// clear timeout
      TimeoutTM [Xidx] = 0;
// set start cycle if needed
      if (XingOcc[Xidx] == false){
        // if a surprise
        XingOcc   [Xidx] = true;
        XingPhase [Xidx] = XingStrtBell;
        NxtPhaseTM[Xidx] = millis();
  	    // ignore exit in either direction
  	    SetPartners (East, StatIgn);
  	    SetPartners (West, StatIgn);
        }// surprise
      }// center occupied
      else {// center cleared
        XingOcc   [Xidx] = false;
        XingPhase [Xidx] = XingRaiseGate;
        if (Crossing[Xidx].GateType == NoGate)
          {NxtPhaseTM[Xidx] = millis();}
          else
          {NxtPhaseTM[Xidx] = millis() + GateDlyMs;}
      }// center cleared
    }// Center changed
  }// for endpoints Sidx
// cleanup all status if sensor timeout & not occupied
  if (XingOcc[Xidx] == false 
      && LstOccRstTM[Xidx] < millis() 
      && LstOccRstTM[Xidx] > 0){
    for (Sidx = 0; Sidx < NumSnsrs; ++Sidx){
        XingDat[Xidx].NowOccStat[Sidx] = StatClr;
    } // for Sidx
    LstOccRstTM[Xidx] = 0;
    #if Trace > 0
      Serial.println  ("Cleared all sensors");
      #endif
  }// cleanup status
 
   if (XingOcc[Xidx] == true)
     {LstOccRstTM[Xidx] = millis() + TimeoutSecs * 1000;}
  
  #if Trace > 2
    Serial.print  ("PS Xing:");
    Serial.print  (XingOcc[Xidx]);
    Serial.print  (" ");
    for (Sidx = 0; Sidx < NumSnsrs; ++Sidx){
    if (Crossing[Xidx].SnsrLst[Sidx] > 0) {//sensor exists
    Serial.print  (SnsrNm(Sidx));
    Serial.print  ("/");
    Serial.print  (Occpied[Sidx]);
    Serial.print  ("/");
    Serial.print  (XingDat[Xidx].Chngd[Sidx]);
    Serial.print  ("/");
    Serial.print  (SenStat[XingDat[Xidx].NowOccStat[Sidx]]);
    Serial.print  (" ");}
    } // for Sidx
    Serial.println  (" ");
    #endif
  return;
} // ProcessSensors
//
#if Trace > 0
// ============= SnsrNm ======================
String SnsrNm (int Idx){
	 return (EWest[Idx/NumTrks] + ":" + Idx%NumTrks);
// ============= SnsrNm ======================
} // SnsrNm
// ============= RptSnsr ======================
void RptSnsr (int Idx){
	Serial.print   (SnsrNm(Idx));
	Serial.println (" set to " + SenStat[XingDat[Xidx].NowOccStat[Idx]]);
// ============= RptSnsr ======================
} // RptSnsr
#endif
// ============= SetPartners ======================
void SetPartners (int Idx, int Stat){
// ============= SetPartners ======================
// set tracks to Stat
// Idx is East/West/Cntr
for (Lidx = Idx; Lidx < Idx + NumTrks; ++Lidx){
   XingDat[Xidx].NowOccStat[Lidx] = Stat;
   #if Trace > 2
      RptSnsr(Lidx);
      #endif
} // for Lidx
} // SetPartners
//
// ============= ProcessPhase ======================
void ProcessPhase (){
// ============= ProcessPhase ======================
  #if Trace > 1
    Serial.print  ("ProcessPhase ");
    Serial.println(XingPhase[Xidx]);
    #endif
//  
if (NxtPhaseTM[Xidx] > 0 
      && NxtPhaseTM[Xidx] <= millis()) {// time to do something
switch (XingPhase[Xidx]){
  case XingStrtBell: // begin cycle
       StartLites();
       break;
  case XingLowerGate: // lites and bell started
       LowerGate();
       break;
  case XingRaiseGate:  // gate is down
       RaiseGate();
       break;
  case XingCleanup:    // gate is up
       StopLites();
       break;
  default:            // should never get here
       #if Trace > 0
         Serial.println("BAD default in XingPhase");
         #endif
       break;
} // end switch
} // if nxtphasetm
}// ProcessPhase
//
// ============= LowerGate ======================
void LowerGate (){
// ============= LowerGate ======================
  for (Lidx = 0; Lidx < NumGates; ++Lidx){
  if (Crossing[Xidx].GateCntl[Lidx] != 0){
  #if Trace > 0
    Serial.print  ("LowerGate " + String(Crossing[Xidx].GateCntl[Lidx]));
    Serial.println(" to "       + String(LGateUD [Lidx][1]));
    #endif
  switch (Crossing[Xidx].GateType){
    case SrvoType:
      #if VSServo
        CrossingGate[Xidx][Lidx].write(map(LGateUD[Lidx][1], 0, 1023, 0, FullArc), ServoSpeed, false);
      #else // jjj
        CrossingGate[Xidx][Lidx].write(map(LGateUD[Lidx][1], 0, 1023, 0, FullArc));
      #endif
      break;
    case DigType:
       digitalWrite(Crossing[Xidx].GateCntl[Lidx], GateDown);
       #if Trace > 1
         Serial.println(" value " + String(GateDown));
       #endif
       break;
    default: break;
  } // switch
  } // if
  }// for
  XingPhase [Xidx] = XingIdle;
  NxtPhaseTM[Xidx] = 0;
}// LowerGate
//
// ============= RaiseGate ======================
void RaiseGate (){
// ============= RaiseGate ======================
  for (Lidx = 0; Lidx < NumGates; ++Lidx){
  if (Crossing[Xidx].GateCntl[Lidx] != 0){
  #if Trace > 0
    Serial.print  ("RaiseGate " + String(Crossing[Xidx].GateCntl[Lidx]));
    Serial.println(" to "       + String(LGateUD [Lidx][0]));
    #endif
  switch (Crossing[Xidx].GateType){
    case SrvoType:
      #if VSServo
        CrossingGate[Xidx][Lidx].write(map(LGateUD[Lidx][0], 0, 1023, 0, FullArc), ServoSpeed, false);
      #else
        CrossingGate[Xidx][Lidx].write(map(LGateUD[Lidx][0], 0, 1023, 0, FullArc));
      #endif
      break;
    case DigType:
      digitalWrite(Crossing[Xidx].GateCntl[Lidx], GateUp);
      #if Trace > 1
         Serial.println(" value " + String(GateUp));
      #endif
       break;
    default: break;
  } // switch
  } // if
  }// for
  XingPhase [Xidx] = XingCleanup;
  NxtPhaseTM[Xidx] = millis() + LiteDlyMs;
}// RaiseGate
//
// ============= StartLites ======================
void StartLites (){
// ============= StartLites ======================
  #if Trace > 1
    Serial.println("StartLites");
    #endif
  if (Crossing[Xidx].FirstLite > 0) {
      digitalWrite(Crossing[Xidx].FirstLite,   LampOn);
      digitalWrite(Crossing[Xidx].FirstLite+1, LampOff);
  } // if FirstLite is defined
  if (Crossing[Xidx].SoundCntl > 0) {
     digitalWrite(Crossing[Xidx].SoundCntl, SndOn);
  } // if SoundCntl is defined
  XingPhase [Xidx] = XingLowerGate;
  NxtPhaseTM[Xidx] = millis() + GateDlyMs;
  NxtFlashTM[Xidx] = millis() + FlashIntvl;
  FlashCycle[Xidx] = false;
}// StartLites
//
// ============= StopLites ======================
void StopLites (){
// ============= StopLites ======================
  #if Trace > 1
    Serial.println("StopLites");
    #endif
  if (Crossing[Xidx].FirstLite > 0) {
      digitalWrite(Crossing[Xidx].FirstLite,   LampOff);
      digitalWrite(Crossing[Xidx].FirstLite+1, LampOff);
  } // if FirstLite is defined
  if (Crossing[Xidx].SoundCntl > 0) {
     digitalWrite(Crossing[Xidx].SoundCntl, SndOff);
  } // if SoundCntl is defined
  XingPhase [Xidx] = XingIdle;
  NxtPhaseTM[Xidx] = 0;
  NxtFlashTM[Xidx] = 0;
  XingOcc   [Xidx] = false;
//  LstOccRstTM[Xidx]= millis() + TimeoutSecs*1000;
}// StopLites
//
// ============= CycleFlash ======================
void CycleFlash (){
// ============= CycleFlash ======================
  if (Crossing[Xidx].FirstLite > 0) {
  if (FlashCycle[Xidx]) {
      digitalWrite(Crossing[Xidx].FirstLite,   LampOn);
      digitalWrite(Crossing[Xidx].FirstLite+1, LampOff);
      FlashCycle[Xidx] = false;
  } 
  else {
      digitalWrite(Crossing[Xidx].FirstLite,   LampOff);
      digitalWrite(Crossing[Xidx].FirstLite+1, LampOn);
      FlashCycle[Xidx] = true;
  } // FlashCycle
  } // if FirstLite is defined
  NxtFlashTM[Xidx] = millis() + FlashIntvl;
}// CycleFlash
//
// ============= RetrieveEE ======================
void RetrieveEE (){
// ============= RetrieveEE ======================
  // initialize from EEProm		
int ii, jj;  
if (LGateFlag == 126){
	LGateFlag      = E_FlagV;
	if (LGateFlag == E_Okay){
    LGateUD [0][0] = E_UpV0;
    LGateUD [1][0] = E_UpV1;
    LGateUD [0][1] = E_DownV0;
    LGateUD [1][1] = E_DownV1;
	}
	else {
		for (ii = 0; ii < NumGates; ++ii){
		for (jj = 0; jj < 2;        ++jj){
       LGateUD [ii][jj] = 511;
		}}// fors
	}// E_Okay
#if Trace > 1
  Serial.print("RetrieveEE:" + String(LGateFlag));
  for (ii = 0; ii < NumGates; ++ii){
  for (jj = 0; jj < 2;        ++jj){
    Serial.print("/" + String(LGateUD [ii][jj]));
  }}
  Serial.println(" ");
#endif
} // LGateFlag
} // RetrieveEE
// ============= RestoreEE ======================
void RestoreEE (){
// ============= RestoreEE ======================
int ii, jj;  
	if (LGateFlag == E_Chngd){
    E_FlagV  =  E_Okay       ;
    E_DownV0 =  LGateUD [0][1];
    E_DownV1 =  LGateUD [1][1];
    E_UpV0   =  LGateUD [0][0];
    E_UpV1   =  LGateUD [1][0];
	  }
 #if Trace > 1
  Serial.print("RestoreEE:" + String(LGateFlag));
  for (ii = 0; ii < NumGates; ++ii){
  for (jj = 0; jj < 2;        ++jj){
    Serial.print("/" + String(LGateUD [ii][jj]));
  }}
  Serial.println(" ");
 #endif
} // RestoreEE
// ============= ProcessPots ======================
void ProcessPots (){
// ============= ProcessPots ======================
int           PotMoved;
int           Idx, Idxx, PotVal, PotInit[NumGates*2];
unsigned long LstTime;

  RetrieveEE();
  RaiseGate();
  for (Idxx = 0; Idxx < NumGates*2; ++Idxx){// read initial pot values
      if (Crossing[Xidx].GatePots[Idxx] != 0){
        PotInit[Idxx] = analogRead(Crossing[Xidx].GatePots[Idxx]);
      }}// if for
  for (Idx = 0; Idx < NumGates; ++Idx){// index gates
    #if Trace > 1
      Serial.println("ProcessPots " + String(Idx));
      #endif
	FlashIt(FlashOn);
	LstTime = millis();
	do {
		for (Idxx = 0; Idxx < 2; ++Idxx){// index pots
      PotMoved = 2;
			if (Crossing[Xidx].GatePots[Idx*2+Idxx] != 0){
			  PotVal = analogRead(Crossing[Xidx].GatePots[Idx*2+Idxx]);
//            Serial.println  ("Pot " + String(Idx) + "," + String(Idxx) + " now " + String(PotVal)+ " orig " + String(PotInit[Idx*2+Idxx]));
			  if (abs(PotVal-LGateUD[Idx][Idxx]) > MinPotMove
			      && abs(PotVal - PotInit[Idx*2+Idxx]) > MinPotMove){
			  	LGateUD [Idx][Idxx] = PotVal;
		  		PotMoved = Idxx;
		  		LstTime = millis();
				  LGateFlag = E_Chngd;
          #if Trace > 1
            Serial.println  ("Pot " + String(Idx) + "," + String(Idxx) + " moved to " + String(PotVal));
            #endif
		      if (PotMoved == 0){RaiseGate();}
		      if (PotMoved == 1){LowerGate();}
			} // MinPotMove
			} // if GatePots
    }// for Idxx Pots
		delay(500);
	} while (LstTime+10000 > millis()); // 10 secs idle time
	if (Idx == 0) {FlashIt(FlashAlt);}
  }// for Idx gates
RestoreEE();
FlashIt(FlashOff);
return;
} // ProcessPots
// ============= FlashIt ======================
void FlashIt (int mode){
// ============= FlashIt ======================
int	ii,jj;
if (Crossing[Xidx].FirstLite > 0){
switch (mode) {
  case FlashOn:
    digitalWrite(Crossing[Xidx].FirstLite,   LampOn);
    digitalWrite(Crossing[Xidx].FirstLite+1, LampOn);
    break;
  case FlashOff:
    digitalWrite(Crossing[Xidx].FirstLite,   LampOff);
    digitalWrite(Crossing[Xidx].FirstLite+1, LampOff);
    break;
  case FlashAlt:
	  for (ii = 0; ii <8; ++ii){
    jj = ii%2 == 0?1:0;
    digitalWrite(Crossing[Xidx].FirstLite+ii%2, LampOn);
    digitalWrite(Crossing[Xidx].FirstLite+jj,   LampOff);
    delay(200);
    digitalWrite(Crossing[Xidx].FirstLite+ii%2, LampOff);
    digitalWrite(Crossing[Xidx].FirstLite+jj,   LampOn);
    delay(200);
    digitalWrite(Crossing[Xidx].FirstLite+jj,   LampOff);
	  }// for
    break;
} // switch
}// lite exists
}// FlashIt
//
// ============= setup ======================
void setup() {
// ============= setup ======================
  int jdx, iflg;
  #if Trace > 0
    Serial.begin(9600);
    Serial.print  (Version);
    Serial.print  ("******* Startup Trace = ");
    Serial.println(Trace);
    #endif
  #if SnsrActiveLED > 0
    pinMode(SnsrActiveLED, OUTPUT); // set relay
    #endif
  // initialize everything

  // now loop for crossings
  for (Xidx = 0; Xidx < NumXings; ++Xidx){
    NxtPhaseTM [Xidx] = 0; 
    NxtFlashTM [Xidx] = 0;
    TimeoutTM  [Xidx] = 0;
    FlashCycle [Xidx] = false;
    XingOcc    [Xidx] = false;
    XingPhase  [Xidx] = XingIdle;
    LstOccRstTM[Xidx] = 0;
  // now loop for sensors
  for (jdx = 0; jdx < NumSnsrs; ++jdx){
    XingDat[Xidx].Smallest   [jdx] = 9999;
    XingDat[Xidx].Largest    [jdx] = 0;
    XingDat[Xidx].HoldOccTM  [jdx] = 0;
    XingDat[Xidx].NowOccStat [jdx] = StatClr;   // current occupied state
    XingDat[Xidx].Chngd      [jdx] = false;    
    if (Crossing[Xidx].SnsrLst [jdx] > 0){
      pinMode(Crossing[Xidx].SnsrLst [jdx],   INPUT);}
    } // for jdx
// now initialize the pins
//  SnsrType, SnsrLst[NumSnsrs], FirstLite, SoundCntl, GateCntl
      if (Crossing[Xidx].FirstLite > 0){
      pinMode(Crossing[Xidx].FirstLite,   OUTPUT);
      pinMode(Crossing[Xidx].FirstLite+1, OUTPUT);
      digitalWrite(Crossing[Xidx].FirstLite,   LampOff);
      digitalWrite(Crossing[Xidx].FirstLite+1, LampOff);
    } // FirstLite
      if (Crossing[Xidx].SoundCntl > 0){
        pinMode(Crossing[Xidx].SoundCntl,      OUTPUT);
        digitalWrite(Crossing[Xidx].SoundCntl, SndOff);
    } // SoundCntl
  LGateFlag = 126; // enable EEProm fetch
  for (Lidx = 0; Lidx < NumGates; ++ Lidx){
  if (Crossing[Xidx].GateCntl[Lidx] != 0){
  switch (Crossing[Xidx].GateType){
    case SrvoType:
      Sidx = Lidx *2; 
      if (Crossing[Xidx].GatePots[Sidx] != 0)
	    	{pinMode(Crossing[Xidx].GatePots[Sidx],     INPUT);}
      if (Crossing[Xidx].GatePots[Sidx+1] != 0)
    		{pinMode(Crossing[Xidx].GatePots[Sidx+1],   INPUT);}
      CrossingGate[Xidx][Lidx].attach(Crossing[Xidx].GateCntl[Lidx]);
      break;
    case DigType:
      pinMode(     Crossing[Xidx].GateCntl[Lidx],   OUTPUT);
      digitalWrite(Crossing[Xidx].GateCntl[Lidx],   GateUp);
      break;
    case NoGate:
      break;
    default: break;
  } // switch
  } // if
  } // for Lidx
  if (Crossing[Xidx].GateType == SrvoType)
    {ProcessPots();}
  //
  // setup done - exercise everything
  #if Trace > 0
    Serial.println  ("Cycle");
  #endif
  StartLites();
  LowerGate();
  CycleStrtMs = millis();
  do {
	delay(400);
    CycleFlash();
  } while (CycleStrtMs + 10000 > millis());
  RaiseGate();
  StopLites();
  }// for Xidx
  SnsrActiveMS = 0;
  Cycles       = 0;
  CycleMs      = 0;
  ResetStrtMts = millis()/60000;
  } // setup =============


// ============= loop ======================
void loop() {
// ============= loop ======================
  int jdx;
  CycleStrtMs = millis();
  Secs = CycleStrtMs / 1000;
  Mts = Secs / 60;

// BEGIN main loop by crossing
for (Xidx = 0; Xidx < NumXings; ++Xidx){
  // check flashing
  if (NxtFlashTM[Xidx] > 0 
    && millis() >= NxtFlashTM[Xidx])
    {CycleFlash();} // Flash active?
  ReadSensors();
  ProcessSensors();
  ProcessPhase();
} // for Xidx main loop processing
// END main loop by crossing
// end of cycle processing

  ThisCycleMs = (millis() - CycleStrtMs);
  CycleMs += ThisCycleMs;
  Cycles +=1;

// check if time to recalibrate sensors
  if ((ResetStrtMts + RecalibrateMts) <= Mts){
    ResetStrtMts = Mts;
    #if Trace > 1
      Serial.print("****** Reset at: ");
      Serial.print(Mts);
      Serial.print(" minutes. Cycles: ");
      Serial.print(Cycles);
      Serial.print(" Avg Ms: ");      
      Serial.println(CycleMs / Cycles);  
      #endif
    for (Xidx = 0; Xidx < NumXings; Xidx++){
    for (jdx = 0;  jdx  < NumSnsrs; jdx++){
       XingDat[Xidx].Largest [jdx] = 0;
       XingDat[Xidx].Smallest[jdx] = 9999;
    } // for jdx
    } // for Xidx
  }// reset

    #if Trace > 2
      Serial.print("Cycle time ");
      Serial.println(ThisCycleMs);
      #endif
      
  #if (MainLoopDlyMS > 0 && Trace > 0)
    if ((ThisCycleMs) < MainLoopDlyMS){delay(MainLoopDlyMS-ThisCycleMs);}
    #endif
  if (SnsrActiveLED > 0 && (SnsrActiveMS + 4000) < millis())
      {digitalWrite(SnsrActiveLED, LOW);}

}// main loop ======================
