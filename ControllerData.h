#ifndef CONTROLLERDATA_H
#define CONTROLLERDATA_H

#include <stdint.h>

#include "HFCommand.h"
#include "ConcurrentQueue.h"

#include "RcPortValues.h"

class ControllerData
{
    public:
        ControllerData();
        virtual ~ControllerData();

        int16_t JoyX;
        int16_t JoyY;
        int16_t JoyZ;
        bool JoyButton;

        int16_t RemoteJoyX;
        int16_t RemoteJoyY;
        int16_t RemoteJoyZ;
        bool RemoteJoyButton;

        float RoiLatitude;
        float RoiLongitude;
        float RoiAltitude;
        bool NewRoi;

        int16_t counter;

        void QueueCommand(HFCommand* hfCommand);
        void ClearCommandQueue();
        HFCommand* GetFirstCommand();

        RcPortValues m_auxRcPort;

        int32_t LedState;
        int32_t LaserState;
        
        int32_t GpsOn;

        void WriteCommandFile();
        
        int32_t gpsDeniedDefaultLatitude;
        int32_t gpsDeniedDefaultLongitude;

    protected:
    private:

        ConcurrentQueue<HFCommand*> m_commandQueue;
};

#endif // CONTROLLERDATA_H



