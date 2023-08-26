#ifndef BAROCONNECTION_H
#define BAROCONNECTION_H

#include "DeviceConnection.h"

#include "BaroData.h"


class BaroConnection : public DeviceConnection
{
    public:
        BaroConnection();
        virtual ~BaroConnection();

        void RequestDataStream(SourceDeviceType device, uint8_t streamId);
        bool DataStreamValid();
        bool PressureReceived();

        void AttachBaroData(BaroData* baroData);

    protected:
        void HandleHeartbeat(mavlink_message_t* msg);
        void HandleScaledPressure(mavlink_message_t* msg);

        mavlink_heartbeat_t             m_heartbeat;
        mavlink_scaled_pressure_t       m_scaledPressure;

        bool m_receivedScaledPressureMsg;

        BaroData* m_baroData;

    private:
};

#endif // BAROCONNECTION_H
