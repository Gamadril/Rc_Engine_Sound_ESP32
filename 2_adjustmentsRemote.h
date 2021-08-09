 
// Select the remote configuration profile you have:
#define GRAUPNER_MZ_12 // Graupner mz-12 PRO

// CHANNEL LINEARITY SETTINGS  ****************************************************************************************************************

 //#define EXPONENTIAL_THROTTLE // Exponential throttle curve. Ideal for enhanced slow speed control in crawlers
 //#define EXPONENTIAL_STEERING // Exponential steering curve. More steering accuracy around center position

// CONFIGURATION PROFILES *********************************************************************************************************************
/*
  // Channel settings -----
  // Channnel assignment
  // Assign your remote channels (the green ones in the excel sheet "adjustmentsRemote.xlsx") and the sound controller channels (the blue ones)
  // for each function. Depending on your wiring and communication mode, not all channels are usable.
  // Example for Flysky:  -> connect receiver CH6 with sound controller CH2 etc. Also make sure, you plug in your PWM wires accordingly

  // Assign the remote channels and the sound controller channels (change the remote channel numbers in #define accordingly)

  // Channels reversed or not: select reversed or not by changing true / false for each channel (if the channel direction is wrong)

  // Channels auto zero adjustment or not: select auto zero calibration or not by changing true / false for each channel.
  // (don't use it for channels without spring centered neutral position @ 1500 microseconds, for potentiometers or for switches)

  // Channels signal range calibration -----
  const uint16_t pulseNeutral = 30; // 1500 +/- this value (around 30) is the neutral range
  const uint16_t pulseSpan = 480; // in theory 500 (1500 center position +/-500 = 1000 - 2000ms) usually about 480

  // Automatic or manual modes -----
  //#define AUTO_LIGHTS // Lights controlled by engine state or controller CH5
  //#define AUTO_ENGINE_ON_OFF // Engine switching on / off automatically via throttle stick and timer or manually by controller CH5
  //#define AUTO_INDICATORS // Indicators triggered automatically by steering angle or manually by controller CH6
*/

// Graupner mz-12 PRO remote configuration profile ---------------------------------------------------------------------------------------------------
#ifdef GRAUPNER_MZ_12

// Channel assignment (use NONE for non existing channels!)
// Remote channel #######   // Sound controller channel ##########################################
#define STEERING 4          // CH1 steering
#define GEARBOX 5           // CH2 3 position switch for gearbox
#define THROTTLE 1          // CH3 throttle & brake
#define HORN 7              // CH4 horn
#define FUNCTION_R 3        // CH5 jake brake, high / low beam, headlight flasher, engine on / off
#define FUNCTION_L 2        // CH6 indicators, hazards
#define POT2 8              // CH7 pot 2
#define MODE1 6             // CH8 mode 1 switch
#define MODE2 NONE             // CH9 mode 2 switch
#define MOMENTARY1 NONE     // CH10
#define HAZARDS 9        // CH11

// Channels reversed or not
boolean channelReversed[13] = {
  false, // CH0 (unused)
  false, // CH1
  false, // CH2
  false, // CH3
  false, // CH4
  false, // CH5
  false, // CH6
  false, // CH7
  false, // CH8
  false, // CH9
  false, // CH10
  false, // CH11
  false  // CH12
}; 

// Channels auto zero adjustment or not (don't use it for channels without spring centered neutral position, switches or unused channels)
boolean channelAutoZero[13] = {
  false, // CH0 (unused)
  false,  // CH1
  false,  // CH2
  false,  // CH3
  false,  // CH4
  false, // CH5
  false, // CH6
  false, // CH7
  false, // CH8
  false, // CH9
  false, // CH10
  false, // CH11
  false  // CH12
}; 

// Channels signal range calibration -----
const uint16_t pulseNeutral = 30;
const uint16_t pulseSpan = 480;

// Automatic or manual modes -----
//#define AUTO_ENGINE_ON_OFF
#define AUTO_INDICATORS

#endif
