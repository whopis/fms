#ifndef SKYBOX_H
#define SKYBOX_H


#include "SkyBoxData.h"
#include "SkyBoxConnection.h"

#include "ReelData.h"

class SkyBox
{
    public:
        SkyBox();
        virtual ~SkyBox();

        SkyBoxData Data;

        void AttachConnection(SkyBoxConnection* skyBoxConnection);

        unsigned int GetHeartbeatCount();
        bool DataStreamValid();

        bool IsConnectionActive(int maxSeconds);


        void SendSetToFlightPosition();
        void SendSetToStoragePosition();
        void LockerOpen();
        void LockerClose();
        void SendEnableIPS(bool enable);
        void SendEnableReel(bool enable);


        bool IsInFlightPosition();
        bool IsInStoragePosition();

        void SetLockerMode(bool lockerMode);

        void SendReelResetTetherPos();
        void SendReelResetTetherPos(int pos);
        void SetReelResetPosition(int pos);
        int GetReelResetPosition();

    protected:

        SkyBoxConnection* m_skyBoxConnection;

        bool m_lockerMode;

    private:
};

#endif // SKYBOX_H
