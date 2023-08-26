#ifndef GPSCONNECTION_H
#define GPSCONNECTION_H

#include "DeviceConnection.h"

#include "GPSData.h"


class GPSConnection : public DeviceConnection
{
    public:
        GPSConnection();
        virtual ~GPSConnection();

        void RequestDataStream(SourceDeviceType device, uint8_t streamId);
        bool DataStreamValid();
        bool PressureReceived();
        bool Pressure2Received();

        void AttachGpsData(GPSData* gpsData);

    protected:
        void HandleHeartbeat(mavlink_message_t* msg);
        void HandleGpsRawInt(mavlink_message_t* msg);
        void HandleGlobalPositionInt(mavlink_message_t* msg);
        void HandleScaledPressure(mavlink_message_t* msg);
        void HandleScaledPressure2(mavlink_message_t* msg);

        mavlink_heartbeat_t             m_heartbeat;
        mavlink_gps_raw_int_t           m_gpsRawInt;
        mavlink_global_position_int_t   m_globalPositionInt;
        mavlink_scaled_pressure_t       m_scaledPressure;
        mavlink_scaled_pressure2_t       m_scaledPressure2;

        int32_t m_latitude;
        int32_t m_longitude;
        int32_t m_altitude;
        uint16_t m_heading;

        uint16_t m_hdop;
        uint16_t m_vdop;
        uint8_t m_fixType;
        uint8_t m_satellites;

        bool m_receivedGlobalPositionMsg;
        bool m_receivedGPSRawMsg;
        bool m_receivedScaledPressureMsg;
        bool m_receivedScaledPressure2Msg;

        GPSData* m_gpsData;

    private:
};

#endif // GPSCONNECTION_H
