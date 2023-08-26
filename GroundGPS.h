#ifndef GROUNDGPS_H
#define GROUNDGPS_H


#include "GPSData.h"
#include "GPSConnection.h"

class GroundGPS
{
    public:
        GroundGPS();
        virtual ~GroundGPS();

        GPSData Data;

        void AttachConnection(GPSConnection* gpsConnection);
        void RequestDataStream();

        unsigned int GetHeartbeatCount();
        bool DataStreamValid();

        bool IsConnectionActive(int maxSeconds);
        bool PressureReceived();


    protected:

        GPSConnection* m_gpsConnection;

    private:
};

#endif // GROUNDGPS_H
