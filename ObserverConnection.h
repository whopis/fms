#ifndef OBSERVERCONNECTION_H
#define OBSERVERCONNECTION_H

#include "DeviceConnection.h"
#include "ObserverData.h"


class ObserverConnection : public DeviceConnection
{
    public:
        ObserverConnection();
        virtual ~ObserverConnection();


        bool DataStreamValid();

        void AttachObserverData(ObserverData* observerData);

        //bool IsLandingRequested();
        bool IsLowerAltitudeRequested();
        int GetTetherLength();

    protected:
        void HandleHeartbeat(mavlink_message_t* msg);


        mavlink_heartbeat_t         m_heartbeat;


        ObserverData* m_observerData;

        //bool m_landingRequested;
        bool m_lowerAltitudeRequested;
        int m_tetherLength;

    private:





};

#endif // OBSERVERCONNECTION_H
