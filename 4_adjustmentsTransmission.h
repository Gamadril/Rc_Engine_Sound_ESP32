/* General TRANSMISSION SETTINGS ************************************************************************************************
 *  
 * Most transmission settings like automatic, double clutch etc. are done in the vehicle configuration files in the /vehicles/ directory.
 * 
 */

//#define TRANSMISSION_NEUTRAL Allows to put the transmission in neutral. This can't be used, if the "Mode 1" button is used for other stuff!
#define TRANSMISSION_NEUTRAL

uint16_t maxClutchSlippingRpm = 300; // The clutch will never slip above this limit! (about 300) 500 for vehicles like locomotives
// and the Kirovets tractor with hydrostatic or electric drive! Mainly required for "VIRTUAL_3_SPEED" mode

// In some cases we want a faster reverse acceleration for automatic transmission vehicles. Around 170% for Landy
uint16_t automaticReverseAccelerationPercentage = 100;

// Automatic transmission with overdrive (lower RPM in top gear, gear ratio lower than 1:1, 4 & 6 speed only)
#define OVERDRIVE // This is usually on, but don't use it for double clutch transmissions!
