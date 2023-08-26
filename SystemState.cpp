#include "SystemState.h"

#include <string.h>


char stateString[40];

const char* SystemStateToString(systemState_t state)
{
    switch(state)
    {
        case notSet:
            strncpy(stateString, "Not Set", 40);
            break;
        case demoMode:
            strncpy(stateString, "Demo Mode", 40);
            break;
        case startup:
            strncpy(stateString, "Startup", 40);
            break;
        case connectToController:
            strncpy(stateString, "Connecting to Controller", 40);
            break;
        case connectToSkyBox:
            strncpy(stateString, "Connecting to SkyBox", 40);
            break;
        case connectToCraft:
            strncpy(stateString, "Connecting to Craft", 40);
            break;
        case connectToGroundGPS:
            strncpy(stateString, "Connecting to Ground GPS", 40);
            break;
        case acquiringCraftGPS:
            strncpy(stateString, "Acquiring Craft GPS", 40);
            break;
        case acquiringControllerGPS:
            strncpy(stateString, "Acquiring Controller GPS", 40);
            break;
        case readySkyBoxClosed:
            strncpy(stateString, "Ready SkyBox Closed", 40);
            break;
        case closingSkyBoxFromReady:
            strncpy(stateString, "SkyBox Closing from Ready", 40);
            break;
        case readyToArm:
            strncpy(stateString, "Ready to Arm", 40);
            break;
        case arming:
            strncpy(stateString, "Arming", 40);
            break;
        case armed:
            strncpy(stateString, "Armed", 40);
            break;
        case launch:
            strncpy(stateString, "Launching", 40);
            break;
        case flightHold:
            strncpy(stateString, "Flight Hold", 40);
            break;
        case flightTranslate:
            strncpy(stateString, "Flight Translate", 40);
            break;
        case flightFollow:
            strncpy(stateString, "Flight Follow", 40);
            break;
        case land:
            strncpy(stateString, "Landing", 40);
            break;
        case failsafe:
            strncpy(stateString, "Failsafe", 40);
            break;
        case safetyStart:
            strncpy(stateString, "Safety Start", 40);
            break;
        case lowVoltage:
            strncpy(stateString, "Low Voltage", 40);
            break;
        case error:
            strncpy(stateString, "Error", 40);
            break;
        default:
            strncpy(stateString, "Unknown", 40);
            break;
    }

    return stateString;
}


