#ifndef SKYBOXDATA_H
#define SKYBOXDATA_H

#include <stdint.h>

#include "BoxcarFilter.h"
#include "ReelData.h"

class SkyBoxData
{
    public:
        SkyBoxData();
        virtual ~SkyBoxData();

        bool ipsEnabled;
        bool reelEnabled;

        int rollTopPosition;
        int scissorLiftPosition;

        int errorMask;
        int moveState;
        int moveTimeRemaining;

        int lockerPosition;



        bool IsSkyBoxInFlightPosition();
        bool IsSkyBoxInStoragePosition();

        bool IsLockerOpen();
        bool IsLockerClosed();
        
        ReelData m_reelData;



    protected:


    private:
};


#endif // SKYBOXDATA_H
