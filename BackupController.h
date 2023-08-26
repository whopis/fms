#ifndef BACKUPCONTROLLER_H
#define BACKUPCONTROLLER_H

#include "ControllerData.h"
#include "BackupControllerConnection.h"

class BackupController
{
    public:
        BackupController();
        virtual ~BackupController();

        ControllerData* Data;

        void AttachControllerData(ControllerData* controllerData);
        void AttachConnection(BackupControllerConnection* backupControllerConnection);


        unsigned int GetHeartbeatCount();
        bool DataStreamValid();

        bool IsConnectionActive(int maxSeconds);

        void SendGpsState(bool craft, uint8_t fixType, uint8_t satellites, uint16_t hdop, uint16_t vdop);

        void SendGlobalPosition(bool craft, uint32_t lat, uint32_t lon, uint32_t alt, uint16_t hdg);



        BackupControllerConnection* m_backupControllerConnection;

    protected:


    private:
};

#endif // BACKUPCONTROLLER_H

