#include "SkyBoxData.h"




#include "HFLogger.h"

SkyBoxData::SkyBoxData()
{
    //ctor
    ipsEnabled = false;
    reelEnabled = false;

    rollTopPosition = -1;
    scissorLiftPosition = -1;

    errorMask = 0;
    moveState = 0;
    moveTimeRemaining = 0;

    lockerPosition = 1;
}

SkyBoxData::~SkyBoxData()
{
    //dtor
}

bool SkyBoxData::IsSkyBoxInFlightPosition()
{
    bool inPosition = true;

    if (rollTopPosition != 2)
        inPosition = false;

    if (scissorLiftPosition != 0)
        inPosition = false;

    if (moveState != 10)
        inPosition = false;

    return inPosition;
}


bool SkyBoxData::IsSkyBoxInStoragePosition()
{
    bool inPosition = true;

    if (rollTopPosition != 0)
        inPosition = false;

    if (scissorLiftPosition != 2)
        inPosition = false;

    if (moveState != 10)
        inPosition = false;

    return inPosition;
}


bool SkyBoxData::IsLockerOpen()
{
    bool inPosition = true;

    //if (lockerPosition != 0)
    //    inPosition = false;
    // with new locker, there are alternate 'open' positions
    switch (lockerPosition)
    {
        case 0:   // nominal unlocked position UNLOCK_A1B1:
        case 11:  // non-optimal unlocked position UNLOCK_A1B2:
        case 12:  // non-optimal unlocked position UNLOCK_A2B1:
        case 13:  // non-optimal unlocked position UNLOCK_A2B2:
            inPosition = true;
            break;
        case 1:     // moving (A mid : B mid)
        case 2:     // nominal locked position (A locked : B locked )
        case 21:    // locked  (A locked : B unlocked primary)
        case 22:    // locked  (A locked : B unlocked secondary)
        case 23:    // locked  (A unlocked primary: B locked)
        case 24:    // locked  (A unlocked secondary: B locked)
        case 31:    // mid  (A locked : B mid)
        case 32:    // mid  (A unlocked primary : B mid)
        case 33:    // mid  (A unlocked seconadry : B mid)
        case 34:    // mid  (A mid : B locked)
        case 35:    // mid  (A mid : B unlocked primary)
        case 36:    // mid  (A mid : B unlocked secondary)
        //case 3:  // UNLOCKED_FAULT (superceded by individual non-optimal unlock positions (11, 12, 13)
        //case 4:  // ERROR - servos done moving but not at desired position (failed to lock a servo)
        default:
            inPosition = false;
            break;
    }
    return inPosition;
}

bool SkyBoxData::IsLockerClosed()
{
    bool inPosition = true;

    //if (lockerPosition != 2)
    //    inPosition = false;
    switch (lockerPosition)
    {
        case 2:     // nominal locked position (A locked : B locked )
        case 21:    // locked  (A locked : B unlocked primary)
        case 22:    // locked  (A locked : B unlocked secondary)
        case 23:    // locked  (A unlocked primary: B locked)
        case 24:    // locked  (A unlocked secondary: B locked)
            inPosition = true;
            break;
        case 0:   // nominal unlocked position UNLOCK_A1B1:
        case 11:  // non-optimal unlocked position UNLOCK_A1B2:
        case 12:  // non-optimal unlocked position UNLOCK_A2B1:
        case 13:  // non-optimal unlocked position UNLOCK_A2B2:
        case 1:     // moving (A mid : B mid)
        case 31:    // mid  (A locked : B mid)
        case 32:    // mid  (A unlocked primary : B mid)
        case 33:    // mid  (A unlocked seconadry : B mid)
        case 34:    // mid  (A mid : B locked)
        case 35:    // mid  (A mid : B unlocked primary)
        case 36:    // mid  (A mid : B unlocked secondary)
        default:
            inPosition = false;
            break;
    }
    return inPosition;
}




