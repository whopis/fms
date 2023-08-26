#ifndef SYSTEMSTATE_H
#define SYSTEMSTATE_H

typedef enum systemState
{
    notSet                  = 0x0000,
    demoMode                = 0x0010,
    startup                 = 0x1000,
    connectToController     = 0x2000,
    connectToSkyBox         = 0x2050,
    connectToCraft          = 0x2100,
    connectToGroundGPS      = 0x2200,
    connectToGroundBaro     = 0x2300,
    acquiringCraftGPS       = 0x4000,
    acquiringControllerGPS  = 0x5000,
    readySkyBoxClosed       = 0x5100,
    closingSkyBoxFromReady  = 0x5200,
    readyToArm              = 0x6000,
    arming                  = 0x7000,
    armed                   = 0x8000,
    launch                  = 0x9000,
    flightHold              = 0xa000,
    flightTranslate         = 0xa100,
    flightFollow            = 0xa200,
    land                    = 0xb000,
    failsafe                = 0xfc00,
    safetyStart             = 0xfd00,
    lowVoltage              = 0xfe00,
    error                   = 0xff00,
    killCraft               = 0xdead
} systemState_t;




typedef enum deviceConnectionState
{
    noConnection            =   0x0000,
    socketConnected         =   0x0010,
    requestingDataStream    =   0x0020,
    waitingForValidData     =   0x0030,
    initializingChannels    =   0x0040,
    connected               =   0x0050,
    lostConnection          =   0x0060
} deviceConnectionState_t;



typedef enum generalState
{
    uninitialized           =   0x0000,
    good                    =   0x0010,
    warning                 =   0x0020,
    critical                =   0x0030
} generalState_t;


typedef enum flightMode
{
    flightmode_stabilize    = 0,
    flightmode_acro         = 1,
    flightmode_alt_hold     = 2,
    flightmode_auto         = 3,
    flightmode_guided       = 4,
    flightmode_loiter       = 5,
    flightmode_rtl          = 6,
    flightmode_circle       = 7,
    flightmode_land         = 9,
    flightmode_of_loiter    = 10,
    flightmode_drift        = 11,
    flightmode_sport        = 13,
    flightmode_flip         = 14,
    flightmode_autotune     = 15,
    flightmode_poshold      = 16
} flightMode_t;


const char* SystemStateToString(systemState_t state);


#endif //SYSTEMSTATE_H
