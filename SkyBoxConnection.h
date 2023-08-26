#ifndef SKYBOXCONNECTION_H
#define SKYBOXCONNECTION_H


#include "DeviceConnection.h"

#include "SkyBoxData.h"
#include "ReelData.h"


class SkyBoxConnection : public DeviceConnection
{
    public:
        SkyBoxConnection();
        virtual ~SkyBoxConnection();

        bool DataStreamValid();

        void AttachSkyBoxData(SkyBoxData* skyBoxData);
//        void AttachReelData(ReelData* reelData);

    protected:
        void HandleHeartbeat(mavlink_message_t* msg);
        void HandleCommandLong(mavlink_message_t* msg);


        void HandleCmdUser4(mavlink_command_long_t commandLong);


        mavlink_heartbeat_t             m_heartbeat;


        SkyBoxData* m_skyBoxData;

    private:
};


#endif // SKYBOXCONNECTION_H
