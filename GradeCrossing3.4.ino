// GradeCrossing controller
//
// A Nano-based sketch to control one or more grade crossings
// -- primarily for the flashing lights
// -- with optional bell and gate controls
// -- gate may be digital or servo controlled
// -- Requires minimum of 3 track sensors - Left/Right/center
//
// *************************************************************************
// Revision history:
#define Version "GradeCrossing 3.4 2025/04/07 11:30"
//   2025/04/08 J. Schmidt 3.4 install NN 
//   2025/04/07 J. Schmidt 3.4 install CV 
//   2025/02/03 J. Schmidt 3.4 fix timeouts
//   2025/01/24 J. Schmidt 3.3 Increase MaxAlertSecs for Nicasio Northern
//   2024/08/05 J. Schmidt 3.3 Add definitions for Nicasio Northern
//   2024/04/04 J. Schmidt 3.3 Rewrite analog sensor processing, sensitivity, & timing
//   2024/04/04 J. Schmidt 3.2 Fixes & Updates to sensitivity & timing
//   2023/04/25 J. Schmidt 3.1a Adjust RecalibrateMts to 6 hours
//   2022/10/18 J. Schmidt 3.1 Modify trace criteria 
//   2022/08/10 J. Schmidt 3.1 Restructure
//   2022/08/04 J. Schmidt 3.0a Add WaitABit for exit processing
//   2022/07/08 J. Schmidt 3.0 Update gate/servo processing with EEProm
//                             Simplify exit processing
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
// ****************** Trace & Debug
// 0 - no trace; 1 - events; 2 - flow and data
//   3 - even more
#define Trace 0
//
#if Trace > 0
  // delay in ms for main loop sampling 
  // -- if tracing/debugging
  #define MainLoopDlyMS 5000
  String ERight[3]  = {"Left","Right"," Cntr"};
  String SenStat[6] = {"no","Clear","Entry","Occupied","Exit","Exiting"};
  String Phases[7]  = {"no","XingClear","XingStrtBell","XingLowerGate","XingRaiseGate","XingCleanup","XingBusy"};
  #endif 
// *************************************************************************
// DO NOT CHANGE THE DEFINES BELOW
// *************************************************************************
#define VSServo true
#if VSServo
  #include <VarSpeedServo.h>
  // speed 10 is slow-ish
  #define ServoSpeed 10
#else
  #include <Servo.h>
#endif
#include "EEPROM-Storage.h"
// *** Global definitions - dont change
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
// Sensor types for in-track sensors: digital or analog
#define AlgType   1
#define DigType   2
// Crossing gate type:
#define NoGate    0
// DigType for on/off control
// SrvoType for servo-controlled gates
#define SrvoType  3
// define sensor groups
#define Left  0
#define Right  NumTrks
#define Cntr  NumTrks * 2
// define setup flash modes
#define FlashOn    1 
#define FlashOff   2
#define FlashAlt   3
// ---OccStat values
// - sensor is not occupied
#define StatClr     1
// - sensor is occupied
#define StatOcc     2
// - phase is new entry
#define StatEnter   3
// - ignore sensor when exiting train covers
#define StatExit    4
#define StatExiting 5
// Phases
// no activity
#define XingClear     1
// start lites and bell 
#define XingStrtBell  2
// drop the gate
#define XingLowerGate 3 
// raise the gate
#define XingRaiseGate 4 
// stop lites & bell & clean up
#define XingCleanup   5
// busy lites and ringing bells
#define XingBusy      6 
// *************************************************************************
// Timing definitions
// |-WaitABitSecs-|B4 Start signal
//                |GateDlyMs|Delay for gate drop
//                |--ShortTripSecs---|Clear signal if crossing not hit
//                                      |GateDlyMs|Delay for gate raise
//                                                |LiteDlyMs|Delay sig stop
//                                      |-----------ExitSecs-------|Cleanup
// LS.................................CS.................................RS
// Left sensors                 Crossing sensors               Right sensors
//
// WaitABitSecs - delay from distant sensor hit to signal starts
// GateDlyMS - delay between lites start and gates drop
// LiteDlyMS - delay between gates raise and lites stop
// ShortTripSecs - timeout if distant sensor hit but train stops short of crossing
// ExitSecs  - ignore exit sensor 
//
// *************************************************************************
// USER TUNEABLE Defines VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
// *************************************************************************
/* 
// DEFINE THE ACTIVE CROSSINGS - DEFAULT
#define NumXings 1 // Number of grade crossings
#define NumTrks  2 // each crossing - up to NumTrks tracks, up to 3 sensors per track
#define NumGates 2 // number of gate controls per crossing
*/
// /*
// Define the active crossings - NumXings crossings - NICASIO NORTHERN
#define NumXings 1 // Number of grade crossings
#define NumTrks  3 // each crossing - up to NumTrks tracks, up to 3 sensors per track
#define NumGates 2 // number of gate controls per crossing
// */
// for above: Left sensors[NumTrks], Right sensors[NumTrks], crossing sensors[NumTrks]
// high/low defines for digital gate - USER-CHANGEABLE
#define GateDown LOW // HIGH
#define GateUp   HIGH // LOW
// high/low defines for sound - USER-CHANGEABLE
#define SndOff   HIGH
#define SndOn    LOW
// define lampon/off - USER-CHANGEABLE
#define LampOn   LOW
#define LampOff  HIGH
// *************************************************************************
// END USER TUNEABLE Defines ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// *************************************************************************
// Define crossing (XingDef) array elements
// DON'T CHANGE
#define NumSnsrs NumTrks * 3
typedef struct {
  // input sensors analog or digital - DigType - AlgType
  int SnsrType; 
  // sensors (LeftSnsr[NumTrks], RightSnsr[NumTrks], CntrSnsr[NumTrks])
  uint8_t SnsrLst[NumSnsrs];
  // WaitABit - time to delay lights after distant sensor hit
  int WaitABitSecs[NumTrks*2];
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
// *************************************************************************
// /*
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// USER DEFINITION for Basic operation
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// updated 5/12/24
// 1 trk south, 1 trk at crossing, 1 trk north
// DEFINES NumXings 1, NumTrks 2, NumGates 2 above to match
// Define the crossing by the following array
XingDef Crossing [] =
//  SnsrType, LeftSnsr[NumTrks], RightSnsr[NumTrks], CntrSnsr[NumTrks], 
  {  AlgType, A4,  0,          0, A3,           A2,  0,  
// WaitABitSecs array Left wait, Right wait
       0,0,0,0,  
//     FirstLite, SoundCntl, GateCntl, 
       10,         4,         0, 00,    
//     GatePots,    
//     Up,Dwn,Up,Dwn     GateType
       0,0,0,0,     NoGate};
//
// Sensitivity - sensor must have dropped % to trigger
#define Sentivity 40
// if digital track sensors use clear for occupied, true
#define DigInvert true 
// FlashIntvl - ms per flash for crossing lites
#define FlashIntvl 600 
// ShortTripSecs - turn off after distant hit but center not hit
#define ShortTripSecs 30
// ExitSecs - ignore exit timer
#define ExitSecs 60
// GateDlyMs - number of ms after sensing and lites/bell start before gates drop
//           - also number of ms after sensing clears gates go up
#define GateDlyMs 1000. // 2500 
// LiteDlyMs - number of ms after sensing clears that lites and bell stop
#define LiteDlyMs 2000.
// END DEFINITIONS for Basic operation
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// */
/*
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// DEFINITIONS for Nicasio Northern
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// checked & updated 8/5/24 new bootloader
// 3 trk south, 2 trks at crossing, 2 trk north
// CHANGE DEFINES NumXings, NumTrks, NumGates above to match
// delay approach from south 
// A crossing is defined by the following array
XingDef Crossing [] =
//  SnsrType, LeftSnsr[NumTrks], RightSnsr[NumTrks], CntrSnsr[NumTrks], 
  {  AlgType, A0, A1, A2,            A3, A4,  0,          A5,  A7, 0,           
// WaitABitSecs array Left wait, Right wait
       0,0,0,0,0,0,  
//     FirstLite, SoundCntl, GateCntl, 
       2,         4,         0, 0,  
//     GatePots,    
//     Up,Dwn,Up,Dwn     GateType
       0,0,0,0,          NoGate};
//
// Sensitivity - sensor must be within % of lowest to trigger
#define Sentivity 70
// if digital track sensors use clear for occupied, true
#define DigInvert true 
//
// FlashIntvl - ms per flash for crossing lites
#define FlashIntvl 600 
// ShortTripSecs - turn off after distant hit but center not hit
#define ShortTripSecs 15
// ExitSecs - ignore exit timer
#define ExitSecs 40
// GateDlyMs - number of ms after sensing and lites/bell start before gates drop
//           - also number of ms after sensing clears gates go up
#define GateDlyMs 0 // 1500 
// LiteDlyMs - number of ms after sensing clears that lites and bell stop
#define LiteDlyMs 2000. 
// END DEFINITIONS for Nicasio Northern
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
/* 
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// DEFINITIONS for Central Vermont South Coventry
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// updated 8/23/22 note new bootloader
// 2 tracks main+siding south, 1 trk at crossing, 1 trk north
// CHANGE DEFINES NumXings, NumTrks, NumGates above to match
// Define the crossing by the following array
XingDef Crossing [] =
//  SnsrType, LeftSnsr[NumTrks], RightSnsr[NumTrks], CntrSnsr[NumTrks], 
  {  AlgType, A0,  A1,            A2, 0,            A3,    0,           
// WaitABitSecs array Left wait, Right wait
       3,0,3,0, 
//     FirstLite, SoundCntl, GateCntl, 
       2,         4,         0, 0,  
//     GatePots,    
//     Up,Dwn,Up,Dwn     GateType
       0,0,0,0,          NoGate};
//
// Sensitivity - sensor must be within % of lowest to trigger
#define Sentivity 70
// if digital track sensors use clear for occupied, true
#define DigInvert true 
// FlashIntvl - ms per flash for crossing lites
#define FlashIntvl 600 
// ShortTripSecs - turn off after distant hit but center not hit
#define ShortTripSecs 30
// ExitSecs - ignore exit timer
#define ExitSecs 120
// GateDlyMs - number of ms after sensing and lites/bell start before gates drop
//           - also number of ms after sensing clears gates go up
#define GateDlyMs 0 // 1500 
// LiteDlyMs - number of ms after sensing clears that lites and bell stop
#define LiteDlyMs 2000. 
// END DEFINITIONS for Central Vermont South Coventry
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
/*
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// DEFINITIONS for Central Vermont Stafford
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// checked & updated 9/8/22 new bootloader
// 1 trk south, 2 trks at crossing, 1 trk north
// CHANGE DEFINES NumXings, NumTrks, NumGates above to match
// delay approach from south 
// A crossing is defined by the following array
XingDef Crossing [] =
//  SnsrType, LeftSnsr[NumTrks], RightSnsr[NumTrks], CntrSnsr[NumTrks], 
  {  AlgType, A0, 0,             A1,    0,          A3,   A2,               
// WaitABitSecs array Left wait, Right wait
       3,0,3,0, 
//     FirstLite, SoundCntl, GateCntl, 
       2,         4,         0, 0,  
//     GatePots,    
//     Up,Dwn,Up,Dwn     GateType
       0,0,0,0,          NoGate};
//
// Sensitivity - sensor must be within % of lowest to trigger
#define Sentivity 70
// if digital track sensors use clear for occupied, true
#define DigInvert true 
//
// FlashIntvl - ms per flash for crossing lites
#define FlashIntvl 600 
// ShortTripSecs - turn off after distant hit but center not hit
#define ShortTripSecs 30
// ExitSecs - ignore exit timer
#define ExitSecs 120
// GateDlyMs - number of ms after sensing and lites/bell start before gates drop
//           - also number of ms after sensing clears gates go up
#define GateDlyMs 0 // 1500 
// LiteDlyMs - number of ms after sensing clears that lites and bell stop
#define LiteDlyMs 2000. 
// END DEFINITIONS for Central Vermont Stafford
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
/*
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// DEFINITIONS for Central Vermont Monson
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Monson updated 8/11 16:00
// 1 trk south, 2 trks at crossing, 1 trk north
// CHANGE DEFINES NumXings, NumTrks, NumGates above to match
// delay approach from south 0 secs
// approach from south probably exits on trk 2 main north
// A crossing is defined by the following array
XingDef Crossing [] =
//  SnsrType, LeftSnsr[NumTrks], RightSnsr[NumTrks], CntrSnsr[NumTrks], 
  {  AlgType,    A0, 0,            A1, 0,            A2,  A3,     
// WaitABitSecs array Left wait, Right wait
       3,0,6,0, 
//     FirstLite, SoundCntl, GateCntl, 
       2,         4,         0, 0,    
//     GatePots,    
//     Up,Dwn,Up,Dwn     GateType
       0,0,0,0,          NoGate};
// Sensitivity - sensor must be within % of lowest to trigger
#define Sentivity 60
// if digital track sensors use clear for occupied, true
#define DigInvert true 
//
// FlashIntvl - ms per flash for crossing lites
#define FlashIntvl 600 
// ShortTripSecs - turn off after distant hit but center not hit
#define ShortTripSecs 30
// ExitSecs - ignore exit timer
#define ExitSecs 120
// GateDlyMs - number of ms after sensing and lites/bell start before gates drop
//           - also number of ms after sensing clears gates go up
#define GateDlyMs 0 
// LiteDlyMs - number of ms after sensing clears that lites and bell stop
#define LiteDlyMs 2000. 
// END DEFINITIONS for Central Vermont Monson
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/
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
#define HoldOccMS 2000
// Servo motion mapping
#define FullArc   160
#if VSServo
  VarSpeedServo CrossingGate[NumXings][NumGates];
#else
  Servo CrossingGate[NumXings][NumGates];
#endif
// Local Servo defines for gate
int LGateFlag;
int LGateUD   [NumGates][2];
// if SnsrActiveLED is HOME-zero, light the 
//     LED indicated if any sensor is active
#define SnsrActiveLED 13
//#define SnsrActiveLED 0
// timing defines
#define OneSec  1000.
// *************************************************************************
// ******************* Variable definitions DO NOT CHANGE
typedef struct {
  // data arrays for sensors
  int Largest     [NumSnsrs]; // largest analog values seen
  int Chngd       [NumSnsrs]; // sensor changed
  int NowOccStat  [NumSnsrs]; // now occupied state
  int SnsrPhase   [NumSnsrs]; // current phase
  unsigned long HoldOccTM [NumSnsrs] ; // timer for false clears
// Crossing timing phase array & definitions
  int           XingPhase ;   // next phase
  unsigned long ShortTripTM;  // timer for center not hit
  unsigned long ExitTM;       // timer for ignore exit
  unsigned long NxtPhaseTM;   // time for next phase processing
  unsigned long NxtFlashTM;   // time for next lite flash
  boolean       FlashCycle;   // cycle for next lite flash
} XingDatStr;
XingDatStr XingDat [NumXings];
//
// LOOP variables
int           Xidx; // index for Crossings
int           Sidx; // index for sensors within crossings
int           Lidx; // local index
unsigned long CycleStrtMs; // cycle start current time
unsigned long Secs;        // current time in seconds
unsigned long Mts;         // current time in minutes
unsigned long SnsrActiveMS;// expire time for SnsrActiveLED
unsigned long Cycles;      // count of loops
unsigned long CycleMs;     // total ms in cycle
unsigned long ThisCycleMs; // total ms in this cycle
// *************************************************************************
// *************************************************************************
// ============= ReadSensors ======================
// read the sensors and set Occpied
void ReadSensors (){
	boolean dsensor, ledflag;
	ledflag = false;
// ============= ReadSensors ======================
// process digital type    
if (Crossing[Xidx].SnsrType == DigType){
// read digital sensors
for (Sidx = 0; Sidx < NumSnsrs; ++Sidx){
	  XingDat[Xidx].Chngd[Sidx] = false;
      if (Crossing[Xidx].SnsrLst[Sidx] > 0){
        dsensor = digitalRead(Crossing[Xidx].SnsrLst[Sidx]);
        if (DigInvert)
		  {dsensor = (dsensor == HIGH)?StatClr:StatOcc;}// invert
	    if (dsensor == StatOcc) {
			XingDat[Xidx].HoldOccTM[Sidx] = millis();
			ledflag = true;
		    } else {if (XingDat[Xidx].HoldOccTM[Sidx] + HoldOccMS <= millis())
			dsensor = StatClr;}
		if (dsensor != XingDat[Xidx].NowOccStat[Sidx]){
			XingDat[Xidx].NowOccStat[Sidx] = dsensor;
		    XingDat[Xidx].Chngd[Sidx] = true;}
		} // if Crossing[Xidx].SnsrLst[Sidx]> 0
} // for Sidx
}  // DigType

 if (Crossing[Xidx].SnsrType == AlgType) { // read analog sensors
  long int Smpls, Tint, Tpct;
  long int Vals[NumSnsrs];
  unsigned long CycleStrt;
  for (Sidx = 0; Sidx < NumSnsrs; ++Sidx)
    {Vals[Sidx] = 0;
     XingDat[Xidx].Chngd[Sidx] = false;}
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
  //
  for (Sidx = 0; Sidx < NumSnsrs; ++Sidx)
    {if (Crossing[Xidx].SnsrLst[Sidx] > 0){ // active sensor
    Vals[Sidx] /= Smpls; // get average
	// calc min/max
    XingDat[Xidx].Largest[Sidx] = max(XingDat[Xidx].Largest[Sidx], Vals[Sidx]);
    Tpct = 100. - (Vals[Sidx] * 100.)/XingDat[Xidx].Largest[Sidx];
    #if Trace > 1
      Serial.print  ("Read ");
      Serial.print  (SnsrNm(Sidx));
      Serial.print  (" sensor ");
      Serial.print  (Crossing[Xidx].SnsrLst[Sidx]);
      Serial.print  (" hi ");
      Serial.print  (XingDat[Xidx].Largest [Sidx]);
      Serial.print  (" current ");
      Serial.print  (Vals[Sidx]);
      Serial.print  (" %drop ");
      Serial.println  (Tpct);
      #endif  
    if (Tpct > Sentivity) // drop is larger than sensitivity 
        {if (XingDat[Xidx].NowOccStat[Sidx] != StatOcc)
			{XingDat[Xidx].Chngd[Sidx] = true;};
		 XingDat[Xidx].NowOccStat[Sidx] = StatOcc;
		 XingDat[Xidx].HoldOccTM [Sidx] = millis();
		 ledflag = true;
        }
	if (Tpct <= Sentivity && (XingDat[Xidx].HoldOccTM [Sidx] + HoldOccMS) < millis()){
      #if Trace > 1
      Serial.print  ("Clear ");
      Serial.println  (SnsrNm(Sidx));
      #endif
	  XingDat[Xidx].Chngd[Sidx] = true;
	  XingDat[Xidx].NowOccStat[Sidx] = StatClr;}
	  
    #if Trace > 0 
     if (XingDat[Xidx].NowOccStat[Sidx] == StatOcc){
      RptSnsr(Sidx);
      Serial.print  (" % drop ");
      Serial.println  (Tpct);
      }
      #endif
    } // if active sensor
    } // for Sidx
}// read analog sensors
//
if (SnsrActiveLED > 0 && ledflag == true)
  {digitalWrite(SnsrActiveLED, HIGH);
   SnsrActiveMS = millis();
   } // SnsrActiveLED
}// ReadSensors
// *************************************************************************
// *************************************************************************
// ============= ProcessSensors ======================
void ProcessSensors (){
	int CntrOcc, LeftOcc, RightOcc;
	int	NxtAction, NewPhase;
// ============= ProcessSensors ======================

NxtAction = -1;

// set occupied counts
  CntrOcc = LeftOcc = RightOcc = 0;
  for (Sidx = 0; Sidx < NumSnsrs; ++Sidx)
    {if (XingDat[Xidx].NowOccStat[Sidx] == StatOcc) 
	  {if (Sidx >= Cntr) CntrOcc=CntrOcc+1;
       else if (Sidx >= Right) RightOcc=RightOcc+1;
       else if (Sidx >= Left)  LeftOcc=LeftOcc+1;}
    } // for
	 
  #if Trace > 1
    Serial.print  ("Incoming Phase: ");
    Serial.print  (XingDat[Xidx].Phases[XingPhase]);
    Serial.print  (" in ");
    Serial.print  ((XingDat[Xidx].NxtPhaseTM > millis())?XingDat[Xidx].NxtPhaseTM - millis():0);
    Serial.print  (" ShortTripTM in ");
    Serial.print  ((XingDat[Xidx].ShortTripTM > millis())?XingDat[Xidx].ShortTripTM - millis():0);
    Serial.print  (" ExitTM in ");
    Serial.println ((XingDat[Xidx].ExitTM > millis())?XingDat[Xidx].ExitTM - millis():0);
    Serial.println (" ");
    for (Sidx = 0; Sidx < NumSnsrs; ++Sidx){
    if (Crossing[Xidx].SnsrLst[Sidx] > 0) //sensor exists
	     {RptSnsr(Sidx);Serial.println  (" ");}
    } // for Sidx
	#endif

// check endpoints changed
  for (Sidx = 0; Sidx < Cntr; ++Sidx){
    NewPhase   = -1;
	
    if (XingDat[Xidx].Chngd[Sidx]){
	  if (XingDat[Xidx].NowOccStat[Sidx] == StatOcc){ // changed to occupied
	    switch (XingDat[Xidx].SnsrPhase[Sidx]) {
		  case StatClr: 
		    NewPhase = StatEnter;
		    // set timeout, set opposite ignore, start lites
			if (XingDat[Xidx].XingPhase != XingBusy){
	    	  NxtAction = XingStrtBell;
              XingDat[Xidx].NxtPhaseTM  = millis() + Crossing[Xidx].WaitABitSecs[Sidx] * OneSec;
		      XingDat[Xidx].ShortTripTM = millis() + (Crossing[Xidx].WaitABitSecs[Sidx] + ShortTripSecs) * OneSec ;
		      SetPartners ((Sidx<Right)?Right:Left, StatExit);
			} // xing idle 
		    break;
		  case StatExit:
		    NewPhase = StatExiting;
			XingDat[Xidx].ExitTM = 0;
		    break;
		  default:
            #if Trace > 1
              Serial.print  ("Unexpected occ endpoint stat ");
              Serial.print  (RptSnsr[Sidx]);
              Serial.println  (" ");
              #endif
            break;
		} //end switch	  
	  } else { // changed to clear
	    switch (XingDat[Xidx].SnsrPhase[Sidx]) {	
		  case StatEnter:
		    NewPhase = StatClr;
		    break;
		  case StatExiting:
		    NewPhase = StatClr;
		    SetPartners (Left,  StatClr);
		    SetPartners (Right, StatClr);
		    break;
		  default:
            #if Trace > 1
              Serial.print  ("Unexpected clear endpoint stat ");
              Serial.print  (RptSnsr[Sidx]);
              Serial.println  (" ");
              #endif
		    break;
	  } // switch
	  } // changed to clear
    if (NewPhase > 0) {
		XingDat[Xidx].SnsrPhase[Sidx] = NewPhase;
		#if Trace > 0
		   Serial.println  ("Endpoint changing to " + SenStat[NewPhase]);
           RptSnsr(Sidx);
		   Serial.println  (" ");
		   #endif
	    }
	} // Sensor Changed
} // for Sidx endpoints

// check center changed
  for (Sidx = Cntr; Sidx < Cntr + NumTrks; ++Sidx){
   NewPhase = -1;
    if (XingDat[Xidx].Chngd[Sidx]){
	  if (XingDat[Xidx].NowOccStat[Sidx] == StatOcc){ // changed to occupied
		XingDat[Xidx].ShortTripTM = 0;
		XingDat[Xidx].ExitTM = millis() + ExitSecs * OneSec ;
        switch (XingDat[Xidx].SnsrPhase[Sidx]){
		  case StatClr: // unexpected start
		    NewPhase = StatOcc;
		    // set timeout, set opposite ignore, start lites
			if (XingDat[Xidx].XingPhase != XingBusy){
	    	  NxtAction = XingStrtBell;
              XingDat[Xidx].NxtPhaseTM  = millis();
		      SetPartners (Left,  StatExit);
		      SetPartners (Right, StatExit);
			} // xing idle 
		    break;
		  default:
            #if Trace > 1
              Serial.print  ("Unexpected endpoint stat ");
              Serial.print  (RptSnsr[Sidx]);
              Serial.println  (" ");
              #endif
            break;
		} // switch
	  // ^^ changed to occupied
	  } else { // changed to clear
        switch (XingDat[Xidx].SnsrPhase[Sidx]){
		  case StatOcc:
	        NewPhase = StatClr;
		    if (CntrOcc = 1){// no other center occupied
	          if (XingDat[Xidx].XingPhase != XingClear){
	            NxtAction = XingRaiseGate;
                XingDat[Xidx].NxtPhaseTM = millis();}
			  } // CntrOcc
		    break;
		  default:
            #if Trace > 1
              Serial.print  ("Unexpected endpoint stat ");
              Serial.print  (RptSnsr[Sidx]);
              Serial.println  (" ");
              #endif
            break;
		} // switch
	  } // ^^ changed to clear
	
	if (NewPhase != -1) {
		#if Trace > 0
		Serial.print  (" Center changing to " + SenStat[NewPhase]);
	    RptSnsr(Sidx);
		Serial.println ();
		#endif
		XingDat[Xidx].SnsrPhase[Sidx] = NewPhase;
    } // NewPhase changed
    } // stat changed
    if (XingDat[Xidx].NowOccStat[Sidx] == StatOcc){ // occupied update ExitTM
		XingDat[Xidx].ExitTM = millis() + ExitSecs * OneSec ;
	    }
	} // for sidx center sensors

	if (NxtAction != -1) {
	  XingDat[Xidx].XingPhase = NxtAction;	
      #if Trace > 0
        Serial.print  ("New Phase: ");
        Serial.print  (Phases[NxtAction]);
        Serial.print  (" ready in: ");
        Serial.print  ((XingDat[Xidx].NxtPhaseTM > millis())?XingDat[Xidx].NxtPhaseTM - millis():0);
        Serial.println ();
        #endif       
	} // NxtAction != -1
      #if Trace > 0
        Serial.print  ("Short: ");
        Serial.print  ((XingDat[Xidx].ShortTripTM > millis())?XingDat[Xidx].ShortTripTM - millis():0);
        Serial.print  (" exit: ");
        Serial.println  ((XingDat[Xidx].ExitTM > millis())?XingDat[Xidx].ExitTM - millis():0);
        #endif       
  // process Short Trip timers
  if (XingDat[Xidx].ShortTripTM > 0 
      && XingDat[Xidx].ShortTripTM < millis() 
	  && CntrOcc == 0
	  && XingDat[Xidx].XingPhase != XingClear) {
       XingDat[Xidx].XingPhase = XingRaiseGate;	
       XingDat[Xidx].NxtPhaseTM = millis();
       XingDat[Xidx].ShortTripTM = 0;
       #if Trace > 0
         Serial.println  ("ShortTripTM tripped");
	     #endif
	  } // timer tripped
  
  // process Exit timer
  if (XingDat[Xidx].ExitTM > 0 
      && XingDat[Xidx].ExitTM < millis()) {
  for (Sidx = 0; Sidx < Cntr; ++Sidx){
    if (XingDat[Xidx].NowOccStat[Sidx] == StatClr
      && XingDat[Xidx].SnsrPhase[Sidx] == StatExit)
	  {XingDat[Xidx].SnsrPhase[Sidx] = StatClr;} // if
    } // end for
    XingDat[Xidx].ExitTM = 0;
    #if Trace > 0
      Serial.println  ("ExitTM tripped");
	  #endif
    } // if timer tripped
  
  return;
} // ProcessSensors
// *************************************************************************
// *************************************************************************
#if Trace > 0
// ============= SnsrNm ======================
String SnsrNm (int Idx){
	 return (ERight[Idx/NumTrks] + ":" + Idx%NumTrks);
// ============= SnsrNm ======================
} // SnsrNm
// ============= RptSnsr ======================
void RptSnsr (int Idx){
	Serial.print   (SnsrNm(Idx));
	Serial.print   (" phase " + SenStat[XingDat[Xidx].SnsrPhase[Idx]]);
	Serial.print   (" occ "   + SenStat[XingDat[Xidx].NowOccStat[Idx]]);
// ============= RptSnsr ======================
} // RptSnsr
#endif
// ============= SetPartners ======================
void SetPartners (int Idx, int Stat){
// ============= SetPartners ======================
// set tracks to Stat
// Idx is Left/Right/Cntr
for (Lidx = Idx; Lidx < Idx + NumTrks; ++Lidx){
   if (XingDat[Xidx].SnsrPhase[Lidx] != StatExiting
     && XingDat[Xidx].NowOccStat[Lidx] != StatOcc)
      XingDat[Xidx].SnsrPhase[Lidx] = Stat;
   #if Trace > 1
      Serial.print  ("SetPartners ");
      RptSnsr(Lidx);Serial.println  ();
      #endif
} // for Lidx
} // SetPartners
// *************************************************************************
// *************************************************************************
// ============= ProcessPhase ======================
void ProcessPhase (){
// ============= ProcessPhase ======================
if (XingDat[Xidx].NxtPhaseTM > 0 
      && XingDat[Xidx].NxtPhaseTM <= millis()) {// time to do something
  #if Trace > 0
    Serial.print  ("ProcessPhase: ");
    Serial.print  (Phases[XingDat[Xidx].XingPhase]);
    Serial.print  (" in ");
    Serial.print  ((XingDat[Xidx].NxtPhaseTM > millis())?XingDat[Xidx].NxtPhaseTM - millis():0);
    Serial.print  (" short in ");
    Serial.print  ((XingDat[Xidx].ShortTripTM > millis())?XingDat[Xidx].ShortTripTM - millis():0);
        Serial.print  (" exit in ");
    Serial.println  ((XingDat[Xidx].ExitTM > millis())?XingDat[Xidx].ExitTM - millis():0);
    #endif
//  
switch (XingDat[Xidx].XingPhase){
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
  case XingBusy:    // xing is active
       break;
  default:            // should never get here
       #if Trace > 1
         Serial.println("BAD default in XingPhase");
         #endif
       break;
} // end switch
} // if nxtphasetm
}// ProcessPhase
// *************************************************************************
// *************************************************************************
// ============= LowerGate ======================
void LowerGate (){
// ============= LowerGate ======================
  for (Lidx = 0; Lidx < NumGates; ++Lidx){
  if (Crossing[Xidx].GateCntl[Lidx] != 0 
      && Crossing[Xidx].GateType != NoGate){
  #if Trace > 1
    Serial.print  ("LowerGate " + String(Crossing[Xidx].GateCntl[Lidx]));
    Serial.println(" to "       + String(LGateUD [Lidx][1]));
    #endif
  switch (Crossing[Xidx].GateType){
    case SrvoType:
      #if VSServo
        CrossingGate[Xidx][Lidx].write(map(LGateUD[Lidx][1], 0, 1023, 0, FullArc), ServoSpeed, false);
      #else 
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
  XingDat[Xidx].XingPhase = XingBusy;
  XingDat[Xidx].NxtPhaseTM = 0;
}// LowerGate
//
// ============= RaiseGate ======================
void RaiseGate (){
// ============= RaiseGate ======================
  for (Lidx = 0; Lidx < NumGates; ++Lidx){
  if (Crossing[Xidx].GateCntl[Lidx] != 0
      && Crossing[Xidx].GateType != NoGate){
  #if Trace > 1
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
  XingDat[Xidx].XingPhase  = XingCleanup;
  XingDat[Xidx].NxtPhaseTM = millis() + LiteDlyMs;
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
  XingDat[Xidx].XingPhase  = XingLowerGate;
  XingDat[Xidx].NxtPhaseTM = millis() + GateDlyMs;
  XingDat[Xidx].NxtFlashTM = millis() + FlashIntvl;
  XingDat[Xidx].FlashCycle = false;
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
  XingDat[Xidx].XingPhase  = XingClear;
  XingDat[Xidx].NxtPhaseTM = 0;
  XingDat[Xidx].NxtFlashTM = 0;
}// StopLites
//
// ============= CycleFlash ======================
void CycleFlash (){
// ============= CycleFlash ======================
  if (Crossing[Xidx].FirstLite > 0) {
  if (XingDat[Xidx].FlashCycle) {
      digitalWrite(Crossing[Xidx].FirstLite,   LampOn);
      digitalWrite(Crossing[Xidx].FirstLite+1, LampOff);
      XingDat[Xidx].FlashCycle = false;
  } 
  else {
      digitalWrite(Crossing[Xidx].FirstLite,   LampOff);
      digitalWrite(Crossing[Xidx].FirstLite+1, LampOn);
      XingDat[Xidx].FlashCycle = true;
  } // FlashCycle
  } // if FirstLite is defined
  XingDat[Xidx].NxtFlashTM = millis() + FlashIntvl;
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
	} while (LstTime + 10*OneSec > millis()); // 10 secs idle time
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
    Serial.println  ("");
    Serial.print  ("******* Startup Trace = ");
    Serial.println(Trace);
    Serial.print  (Version);
    Serial.println  ("");
    #endif
  #if SnsrActiveLED > 0
    pinMode(SnsrActiveLED, OUTPUT); // set relay
    #endif
  // initialize everything

  // now loop for crossings
  for (Xidx = 0; Xidx < NumXings; ++Xidx){
    XingDat[Xidx].NxtPhaseTM = 0; 
    XingDat[Xidx].NxtFlashTM = 0;
    XingDat[Xidx].ShortTripTM= 0;
    XingDat[Xidx].ExitTM     = 0;
    XingDat[Xidx].FlashCycle = false;
    XingDat[Xidx].XingPhase  = XingClear;
  // now loop for sensors
  for (jdx = 0; jdx < NumSnsrs; ++jdx){
    XingDat[Xidx].Largest    [jdx] = 0;
    XingDat[Xidx].HoldOccTM  [jdx] = 0;
    XingDat[Xidx].SnsrPhase [jdx]  = StatClr;   // phase
    XingDat[Xidx].NowOccStat [jdx] = StatClr;   // current occupied state
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
    Serial.println  ("Exercise everything");
  #endif
  StartLites();
  LowerGate();
  CycleStrtMs = millis();
  do {
	delay(200);
    CycleFlash();
  } while (CycleStrtMs + 3000. > millis());
  RaiseGate();
  StopLites();
  }// for Xidx
  #if Trace > 0
    Serial.println  ("Begin normal cycle");
  #endif
  SnsrActiveMS = 0;
  Cycles       = 0;
  CycleMs      = 0;
  } // setup =============


// ============= loop ======================
void loop() {
// ============= loop ======================
  int jdx;
  CycleStrtMs = millis();
  Secs = CycleStrtMs / OneSec;
  Mts = Secs / 60;

// BEGIN main loop by crossing
for (Xidx = 0; Xidx < NumXings; ++Xidx){
  // check flashing
  if (XingDat[Xidx].NxtFlashTM > 0 
    && millis() >= XingDat[Xidx].NxtFlashTM)
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

    #if Trace > 2
      Serial.print("Cycle time ");
      Serial.println(ThisCycleMs);
      #endif
      
  #if Trace > 0
    if (MainLoopDlyMS > 0)
    {if ((ThisCycleMs) < MainLoopDlyMS){delay(MainLoopDlyMS-ThisCycleMs);}}
    #endif
	
  if (SnsrActiveLED > 0 && (SnsrActiveMS + 4000.) < millis())
      {digitalWrite(SnsrActiveLED, LOW);}

}// main loop ======================
