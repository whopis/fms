#ifndef GPSDATA_H
#define GPSDATA_H

#include <stdint.h>


class GPSData
{
    public:
        GPSData();
        virtual ~GPSData();

        int32_t latitude;
        int32_t longitude;
        uint32_t relative_alt;
        uint16_t heading;

        uint16_t gpsHdop;
        uint16_t gpsVdop;
        uint8_t gpsFixType;
        uint8_t gpsNumberSatellites;

        float pressAbs;
        int16_t temperature;

        float pressAbs2;
        int16_t temperature2;

        void SetRequirements(int satellites, int hdop, bool debugSkipGps, bool baroCompensate);

        bool IsGpsGood();

    protected:

        int m_minSatellites;
        int m_maxHdop;
        bool m_debugSkipGps;
        bool m_baroCompensate;

    private:
};

#endif // GPSDATA_H
