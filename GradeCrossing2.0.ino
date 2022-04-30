// GradeCrossing controller
//
// A Nano-based sketch to control one or more grade crossings
// -- primarily for the flashing lights
// -- with optional bell and gate controls
// -- gate is servo controlled
// -- Requires minimum of 3 sensors - east/west/center
//
// *************************************************************************
// Revision history:
//   2022/04/30 J. Schmidt 2.0 Update gate & servo processing
//                             Optionally use VarSpeedServo
//                             fixed TimeOutSecs starts after WaitABitSecs
//   2022/02/20 J. Schmidt 1.3 Redo sensor sensitivity
//   2022/02/02 J. Schmidt 1.2a Fix GateDly on xing cleared
//   2022/01/25 J. Schmidt 1.2 Renumber types, add NoGate & code for multiple gates; 
//   2022/01/22 J. Schmidt 1.1 Fix start flash; fix refresh of sensors;
//   2022/01/13 J. Schmidt 1.0 Fix WaitABitSecs for Xidx crossings
//   2022/01/08 J. Schmidt 1.0 Add the WaitABitSecs to delay signal after distant hit
//   2021/12/28 J. Schmidt 0.6 Use define for number of tracks - fix get_partner
//   2021/12/27 J. Schmidt 0.5 First operational version
//   2021/12/26 J. Schmidt 0.4 Continue changes for multiple tracks and xing sensor
//   2021/12/18 J. Schmidt 0.3 Begin changes for multiple tracks and xing sensor
//   2021/12/10 J. Schmidt 0.2 Operational with breadboard - servo not tested
//   2021/12/05 J. Schmidt 0.1 Start build
// *************************************************************************
//
#define VSServo true
#if VSServo
  #include <VarSpeedServo.h>
  #define ServoSpeed 10
#else
  #include <Servo.h>
#endif
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
// *************************************************************************
// USER TUNEABLE Defines
// *************************************************************************
// Define the active crossings - NumXings crossings
// each crossing - up to NumTrks tracks, up to 3 sensors per track
// number of gate controls per crossing
#define NumXings 1
#define NumTrks  1
#define NumGates 2
//
// dont change:
#define NumSnsrs NumTrks * 3
// for above: east sensors[NumTrks], west sensors[NumTrks], crossing sensors[NumTrks]
// Sensor types for in-track sensors: digital or analog
#define DigType   2
#define AlgType   1
// Crossing gate type:
#define NoGate    0
#define SrvoType  3
// -- or DigType for gate control
//
// Define crossing (XingDef) array elements
typedef struct {
  // input sensors analog or digital - DigType - AlgType
  int SnsrType; 
  // first sensor address
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
///*
// DEFINITIONS for Central Vermont South Coventry
// main+siding south, 1 trk at crossing, 1 trk north
// A crossing is defined by the following array
XingDef Crossing [NumXings] =
//  SnsrType, EastSnsr[NumTrks], WestSnsr[NumTrks], XingSnsr[NumTrks], 
  {  AlgType, A0,                A2,                A1,               
//     FirstLite, SoundCntl, GateCntl, 
       2,         4,         9, 10,    
//     GatePots,    
//     Up,Dwn,Up,Dwn     GateType
       A6,A7, A6,A7,     SrvoType};
// and the WaitABitSecs array
// East wait, West wait
int WaitABitSecs[NumXings][NumTrks*2] =
  {0,0};
//
// Sensitivity - sensor must be within % of lowest to trigger
#define Sentivity 16
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
//*/
//
/*
// DEFINITIONS for Central Vermont Stafford
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
// */
//
/*
// DEFINITIONS for Central Vermont Monson
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
// */
//
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
//
// Servo defines for gate
int GateDownS = {10};
int GateUpS   = {120};
#define FullArc   160
#if VSServo
  VarSpeedServo CrossingGate[NumXings][NumGates];
#else
  Servo CrossingGate[NumXings][NumGates];
#endif
// high/low defines for gate
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
//
// ******************* Variable definitions
boolean Occpied[NumSnsrs];
typedef struct {
  // data arrays for sensors
  int Smallest   [NumSnsrs]; // smallest analog values seen
  int Largest    [NumSnsrs]; // largest analog values seen
  int NowOccStat [NumSnsrs]; // current occupied state
  boolean Chngd  [NumSnsrs]; // state changed
  unsigned long 
       HoldOccTM [NumSnsrs]; // timing to eliminate false clears
int LstEndSnsr;
} XingDatStr;
//
XingDatStr XingDat [NumXings];
//
// NowOccStat values
// - sensor is not occupied
#define StatClr   0
// - sensor is occupied
#define StatOcc   1
// - ignore sensor until train covers
#define StatIgn   2
// train hit ignore sensor - clear when clear
#define StatClrng 3
//
// timing phase array & definitions
unsigned long TimeoutTM  [NumXings]; // time for dead move timeout
boolean       XingOcc    [NumXings]; // is the crossing occupied
unsigned long LstOccRstTM[NumXings]; // timing to clear ign/clrng
int           XingPhase  [NumXings]; // next phase
unsigned long NxtPhaseTM [NumXings]; // time for next phase processing
unsigned long NxtFlashTM [NumXings]; // time for next lite flash
boolean       FlashCycle [NumXings]; // cycle for next lite flash
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
int           Pidx; // index for partner of endpoint
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
// ============= ProcessSensors ======================
void ProcessSensors (){
// ============= ProcessSensors ======================
  #if Trace > 0
    Serial.print  ("Xing status ");
    Serial.print  (XingOcc[Xidx]);
    Serial.print  (" ");
    Serial.print  (LstOccRstTM[Xidx]);
    Serial.print  (" ");
    Serial.println(millis());
    Serial.print  ("ProcessSensors sensor/changed/value ");
    for (Sidx = 0; Sidx < NumSnsrs; ++Sidx){
    Serial.print  (Sidx);
    Serial.print  ("/");
    Serial.print  (XingDat[Xidx].Chngd[Sidx]);
    Serial.print  ("/");
    Serial.print  (XingDat[Xidx].NowOccStat[Sidx]);
    Serial.print  (" ");
    } // for Sidx
    Serial.println  (" ");
    #endif
// cleanup ign/clrng status
  if (XingOcc[Xidx] == true)
    {LstOccRstTM[Xidx] = millis() + TimeoutSecs * 1000;}
  if (XingOcc[Xidx] == false 
      && LstOccRstTM[Xidx] < millis() 
    && LstOccRstTM[Xidx] > 0){
    for (Sidx = 0; Sidx < Cntr; ++Sidx){
      if (XingDat[Xidx].NowOccStat[Sidx] == StatIgn
          || XingDat[Xidx].NowOccStat[Sidx] == StatClrng){
        XingDat[Xidx].NowOccStat[Sidx] = StatClr;
        LstOccRstTM[Xidx] = 0;
      #if Trace > 0
        Serial.print  ("Cleared ");
        Serial.println(Sidx);
        #endif
        }// if
    } // for Sidx
  }// cleanup status

// process deadman timeout
  if (TimeoutTM[Xidx] > 0 && TimeoutTM[Xidx] < millis()){// active & expired
      #if Trace > 0
        Serial.println("Process deadman timeout");
        #endif
      TimeoutTM [Xidx] = 0;
      XingOcc   [Xidx] = false;
      XingPhase [Xidx] = XingRaiseGate;
      NxtPhaseTM[Xidx] = millis();
      for (Sidx = 0; Sidx < Cntr; ++Sidx){
        // check/clear the ignore flag
        if (XingDat[Xidx].NowOccStat[Sidx] == StatIgn)
           {XingDat[Xidx].NowOccStat[Sidx] = StatClr;}
      } // for Sidx
      }// invoke deadman & clear
//
// check endpoints changed
  for (Sidx = 0; Sidx < Cntr; ++Sidx){
    if (XingDat[Xidx].Chngd[Sidx] == true){
      if (XingDat[Xidx].NowOccStat[Sidx] == StatOcc){
// endpoint became occupied
// set timeout and start lites/bell
      #if Trace > 0
        Serial.print  ("Endpoint now live set deadman ");
        Serial.println(Sidx);
        #endif
      TimeoutTM [Xidx] = millis() + (TimeoutSecs + WaitABitSecs[Xidx][Sidx]) * 1000;
      XingDat[Xidx].LstEndSnsr = Sidx;
      if (XingOcc[Xidx] == false){
      XingPhase [Xidx] = XingStrtBell;
      NxtPhaseTM[Xidx] = millis() + WaitABitSecs[Xidx][Sidx] * 1000;
      XingOcc   [Xidx] = true;
      }// if not occupied
      }// sensor occupied
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
      }// surprise
      // find partner
      GetPartner();
      XingDat[Xidx].NowOccStat[Pidx] = StatIgn;
      LstOccRstTM[Xidx] = millis() + TimeoutSecs * 1000;
      #if Trace > 0
        Serial.print  ("Set endpoint partner to ignore ");
        Serial.println(Pidx);
        #endif
      }// process occupied
      else {// center cleared
      XingOcc   [Xidx] = false;
      XingPhase [Xidx] = XingRaiseGate;
      if (Crossing[Xidx].GateType == NoGate)
         {NxtPhaseTM[Xidx] = millis();}
         else
         {NxtPhaseTM[Xidx] = millis() + GateDlyMs;}
      }// center cleared
    }// Changed
    
  }// for endpoints Sidx
  return;
} // ProcessSensors
//
// ============= GetPartner ======================
void GetPartner (){
// ============= GetPartner ======================
// find endpoint partner to center sensor
// try corresponding first
int evenodd, base;
evenodd = XingDat[Xidx].LstEndSnsr % NumTrks;
base    = XingDat[Xidx].LstEndSnsr / NumTrks;
if (base == East)
  {base = West;}
  else
  {base = East;}
// try corresponding first
Pidx = base + evenodd;
#if Trace > 0
    Serial.print  ("Try sensor ");
    Serial.print  (Pidx);
    Serial.print  (" NowOcc ");
    Serial.println(XingDat[Xidx].NowOccStat[Pidx]);
    #endif
if (XingDat[Xidx].NowOccStat[Pidx] == StatClr
      && Crossing[Xidx].SnsrLst[Pidx] > 0)
   {return;}
   
// unsuccessful - scan all distant sensors  
for (Pidx = base; Pidx < base + NumTrks; ++Pidx){
#if Trace > 0
    Serial.print  ("Try sensor ");
    Serial.print  (Pidx);
    Serial.print  (" NowOcc ");
    Serial.println(XingDat[Xidx].NowOccStat[Pidx]);
    #endif
  if (XingDat[Xidx].NowOccStat[Pidx] == StatClr
      && Crossing[Xidx].SnsrLst[Pidx] > 0)
      {return;}
} // for Pidx
} // GetPartner
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
       if (Occpied[Sidx] == true)
          {Occpied[Sidx] = false;}
       else
          {Occpied[Sidx] = true;}
       }// invert
      } // if Crossing[Xidx].SnsrLst[Sidx]> 0
} // for Sidx
} // process digital sensors
else { // read analog sensors
//
  long int Smpls, Tint;
  long int Vals[NumSnsrs];
  unsigned long CycleStrt;
  for (Sidx = 0; Sidx < NumSnsrs; ++Sidx){
     Vals   [Sidx] = 0;
     XingDat[Xidx].Chngd[Sidx] = false;
     Occpied[Sidx] = false;}
  Smpls = 0;
  CycleStrt = millis();
  do { // do the integration over LoopMS time
    // read all sensors
    for (Sidx = 0; Sidx < NumSnsrs; ++Sidx)
    {if (Crossing[Xidx].SnsrLst[Sidx] > 0)
      {Vals[Sidx] += analogRead(Crossing[Xidx].SnsrLst[Sidx]);}}// for-if
      
    ++Smpls;
    #if DlyInLoop > 0
      delay(DlyInLoop);
      #endif 
  } while (CycleStrt + LoopMS > millis());
  
  // do the integration over LoopMS time

    for (Sidx = 0; Sidx < NumSnsrs; ++Sidx)
    {if (Crossing[Xidx].SnsrLst[Sidx] > 0){
      // active sensor
    Vals[Sidx] /= Smpls;
    if (Vals[Sidx] > XingDat[Xidx].Largest[Sidx]) 
                    {XingDat[Xidx].Largest[Sidx]  = Vals[Sidx];}
    if (XingDat[Xidx].Smallest[Sidx] == 9999)
                    {XingDat[Xidx].Smallest[Sidx] = Vals[Sidx]*.9;}
    if (Vals[Sidx] < XingDat[Xidx].Smallest[Sidx])
                    {XingDat[Xidx].Smallest[Sidx] = Vals[Sidx];}
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
    #if Trace > 1
      Serial.print  ("Read ");
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
      Serial.print  (" <value ");
      Serial.println(Vals[Sidx]);
      #endif
        }// in lower limit 
    } // if Tint
    } // if active sensor
    } // for Sidx
}// process analog sensors
//
for (Sidx = 0; Sidx < NumSnsrs; ++Sidx){
if (Occpied[Sidx] == true){
    #if Trace > 0
      if (Occpied[Sidx]){
        Serial.print  ("### Sensor # ");
        Serial.print  (Sidx);
        Serial.print  (" occ ");
        Serial.println(Crossing[Xidx].SnsrLst[Sidx]);
      }// occupied
      #endif
  XingDat[Xidx].HoldOccTM[Sidx] = millis() + HoldOccMS;
  //
  if (XingDat[Xidx].NowOccStat[Sidx] == StatClr){
    // was clear, set occupied
      XingDat[Xidx].NowOccStat[Sidx] =  StatOcc;
      XingDat[Xidx].Chngd     [Sidx] =  true;
      }// was clear, set occupied
  //
  if (XingDat[Xidx].NowOccStat[Sidx] == StatIgn){
    // was ignore, set to clearing
      XingDat[Xidx].NowOccStat[Sidx] =  StatClrng;
  }// was was ignore, set to clearing
  }// Occpied is true
  //
  else{ 
    // Occpied is false
  if (XingDat[Xidx].HoldOccTM[Sidx] < millis()){
    // holdocc has expired
  if (XingDat[Xidx].NowOccStat[Sidx]   == StatOcc
    || XingDat[Xidx].NowOccStat[Sidx]  == StatClrng){
      XingDat[Xidx].NowOccStat[Sidx]    = StatClr;
      XingDat[Xidx].Chngd     [Sidx]    = true;
  }// if occupied or clearing
  }// if hold time expired
}// Occpied is false
//
if (SnsrActiveLED > 0 && Occpied[Sidx] == true)
  {digitalWrite(SnsrActiveLED, HIGH);
   SnsrActiveMS = millis();
   } // SnsrActiveLED
} // for Sidx
}// ReadSensors
//
// ============= LowerGate ======================
void LowerGate (){
// ============= LowerGate ======================
  for (Lidx = 0; Lidx < NumGates; ++Lidx){
  if (Crossing[Xidx].GateCntl[Lidx] != 0){
  #if Trace > 1
    Serial.print("LowerGate " + String(Crossing[Xidx].GateCntl[Lidx]));
    #endif
  switch (Crossing[Xidx].GateType){
    case SrvoType:
      Sidx = Lidx *2+1; 
      if (Crossing[Xidx].GatePots[Sidx] != 0){
         GateDownS = analogRead(Crossing[Xidx].GatePots[Sidx]);}
      #if VSServo
        CrossingGate[Xidx][Lidx].write(map(GateDownS, 0, 1023, 0, FullArc), ServoSpeed, false);
      #else
        CrossingGate[Xidx][Lidx].write(map(GateDownS, 0, 1023, 0, FullArc));
      #endif
      #if Trace > 1
         Serial.println(" pot " + String(Crossing[Xidx].GatePots[Sidx]) + " value " + String(GateDownS));
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
  #if Trace > 1
    Serial.print("RaiseGate " + String(Crossing[Xidx].GateCntl[Lidx]));
    #endif
  switch (Crossing[Xidx].GateType){
    case SrvoType:
      Sidx = Lidx *2; 
      if (Crossing[Xidx].GatePots[Sidx] != 0){
         GateUpS = analogRead(Crossing[Xidx].GatePots[Sidx]);}
      if (Crossing[Xidx].GatePots[Sidx] != 0){
         GateDownS = analogRead(Crossing[Xidx].GatePots[Sidx]);}
      #if VSServo
        CrossingGate[Xidx][Lidx].write(map(GateUpS, 0, 1023, 0, FullArc), ServoSpeed, false);
      #else
        CrossingGate[Xidx][Lidx].write(map(GateUpS, 0, 1023, 0, FullArc));
      #endif
      #if Trace > 1
         Serial.println(" pot " + String(Crossing[Xidx].GatePots[Sidx]) + " value " + String(GateUpS));
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
  LstOccRstTM[Xidx]= millis() + TimeoutSecs*1000;
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
// ============= setup ======================
void setup() {
// ============= setup ======================
  int jdx, iflg;
  #if Trace > 0
    Serial.begin(9600);
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
    XingDat    [Xidx].LstEndSnsr       
                      = 0;
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
  for (Lidx = 0; Lidx < NumGates; ++ Lidx){
  if (Crossing[Xidx].GateCntl[Lidx] != 0){
  switch (Crossing[Xidx].GateType){
    case SrvoType:
      Sidx = Lidx *2; 
      if (Crossing[Xidx].GatePots[Sidx] != 0){
         pinMode(Crossing[Xidx].GatePots[Sidx],   INPUT);
         GateUpS = analogRead(Crossing[Xidx].GatePots[Sidx]);}
      if (Crossing[Xidx].GatePots[Sidx+1] != 0){
         pinMode(Crossing[Xidx].GatePots[Sidx+1],   INPUT);}
      CrossingGate[Xidx][Lidx].attach(Crossing[Xidx].GateCntl[Lidx]);
      CrossingGate[Xidx][Lidx].write(map(GateUpS, 0, 1023, 0, FullArc));
      break;
    case DigType:
    case NoGate:
      pinMode(     Crossing[Xidx].GateCntl[Lidx],   OUTPUT);
      digitalWrite(Crossing[Xidx].GateCntl[Lidx],   GateUp);
      break;
    default: break;
  } // switch
  } // if
  } // for
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
