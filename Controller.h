#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "ControllerData.h"
#include "ControllerConnection.h"

class Controller
{
    public:
        Controller();
        virtual ~Controller();

        ControllerData Data;

        void AttachConnection(ControllerConnection* ControllerConnection);

        unsigned int GetHeartbeatCount();
        bool DataStreamValid();

        bool IsConnectionActive(int maxSeconds);

        void SendGpsState(bool craft, uint8_t fixType, uint8_t satellites, uint16_t hdop, uint16_t vdop);

        void SendGlobalPosition(bool craft, uint32_t lat, uint32_t lon, uint32_t alt, uint16_t hdg);

        void ClearManualCommands();



        ControllerConnection* m_controllerConnection;

    protected:



    private:
};

#endif // CONTROLLER_H




