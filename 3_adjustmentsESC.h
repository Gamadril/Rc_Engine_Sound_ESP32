/* Recommended ESC & settings

*****************************************************************************
ESC: HOBBYWING 1080 QUICRUN WP Crawler Brushed with the following settings:
Vehicle: TAMIYA trucks with 3 speed transmission
Motor: 540 size, 35 turns, stock pinion

For more details refer to the ESC manual.
Uncommented settings are left on factory preset
1: 3
2: 1 (1=  LiPo, 2 = NIMH battery)
3: 3
4: 3
5: 4
6: 2
7: 9
8: 1 
9: 8 (change it, important)
10: 4
11: 4
12: 5
13: 5 (16KHz = less whining)
14: 1 (be careful here, this will change the BEC voltage!!)
15: 1 (change it, important)
*****************************************************************************
*/

// ESC SETTINGS ******************************************************************************************************

// Drive direction adjustment:
//#define ESC_DIR // uncomment this, if your motor is spinning in the wrong direction

// Top speed adjustment:
// Usually 500 ( = 1000 - 2000 microseconds output or -45° to 45° servo angle) Enlarge it, if your vehicle is too fast
// - Hobbywing 1080 ESC & 35T 540 motor for TAMIYA trucks with 3 speed transmission = 1200
const int16_t escPulseSpan = 500; // 500 = full ESC power available, 1000 half ESC power available etc. 

// Additional takeoff punch:
// Usually 0. Enlarge it, if your motor is too weak around neutral.
// - Hobbywing 1080 ESC & 35T 540 motor for TAMIYA trucks with 3 speed transmission = 0
const int16_t escTakeoffPunch = 0; 

// Additional reverse speed (disconnect & reconnect battery after changing this setting):
// Usually 0. Enlarge it, if your reverse speed is too slow.
// - Hobbywing 1080 ESC & 35T 540 motor for TAMIYA trucks with 3 speed transmission = 0
const int16_t escReversePlus = 0;
