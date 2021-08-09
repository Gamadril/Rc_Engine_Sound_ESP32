/* General TRANSMISSION SETTINGS ************************************************************************************************
 *  
 * Most lights settings are done in the vehicle configuration files in the /vehicles/ directory.
 * 
 */

#define NO_CABLIGHTS // The cablights step in the lights sequence is skipped, if defined
//#define NO_FOGLIGHTS // The foglights step in the lights sequence is skipped, if defined

// All brightness values 0 - 255
uint8_t sideLightsBrightness = 100; // Usually 200, 100 for WPL C44, 50 for Landy, 100 for P407
uint8_t rearlightDimmedBrightness = 30; // tailligt brigthness, if not braking, about 30
uint8_t rearlightParkingBrightness = 5; // 0, if you want the taillights being off, if side lights are on, or about 5 if you want them on
uint8_t reversingLightBrightness = 140; // Around 140, 50 for Landy
