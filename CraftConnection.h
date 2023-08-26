#ifndef CRAFTCONNECTION_H
#define CRAFTCONNECTION_H

#include "DeviceConnection.h"

#include "CraftData.h"

#include "SourceData.h"


class CraftConnection : public DeviceConnection
{
    public:
        CraftConnection();
        virtual ~CraftConnection();

       void AttachCraftData(CraftData* gpsData);


       bool DataStreamValid();

       void RequestDataStream(SourceDeviceType device, uint8_t streamId);
       void OverrideRcChannelsLow(SourceDeviceType device, uint16_t rc1, uint16_t rc2, uint16_t rc3, uint16_t rc4, uint16_t rc5, uint16_t rc6, uint16_t rc7, uint16_t rc8);
       void OverrideRcChannelsHigh(SourceDeviceType device, uint16_t rc1, uint16_t rc2, uint16_t rc3, uint16_t rc4, uint16_t rc5, uint16_t rc6, uint16_t rc7, uint16_t rc8);

    protected:
        void HandleHeartbeat(mavlink_message_t* msg);           // 000  /   0x00
        void HandleSysStatus(mavlink_message_t* msg);           // 001  /   0x01
        void HandleParamValue(mavlink_message_t* msg);          // 022  /   0x16
        void HandleGpsRawInt(mavlink_message_t* msg);           // 024  /   0x16
        void HandleGps2Raw(mavlink_message_t* msg);             // 124  /   0x7C
        void HandleGlobalPositionInt(mavlink_message_t* msg);   // 033  /   0x21
        void HandleRCChannelsRaw(mavlink_message_t* msg);       // 035  /   0x23
        void HandleServoOutputRaw(mavlink_message_t* msg);      // 036  /   0x24
        void HandleStatusText(mavlink_message_t* msg);          // 253  /   0xFD
        void HandleScaledPressure(mavlink_message_t* msg);      // 029  /   0x1C
        void HandleScaledPressure2(mavlink_message_t* msg);     // 137  /   0x89
        void HandleScaledPressure3(mavlink_message_t* msg);     // 143  /   0x8F
        void HandleAutopilotVersion(mavlink_message_t* msg);
        void HandleRangefinder(mavlink_message_t* msg);         // 173  /   0xAD
        void HandleVibration(mavlink_message_t* msg);           // 241  /   0xF1


        mavlink_heartbeat_t             m_heartbeat;

        mavlink_scaled_pressure_t       m_scaledPressure;
        mavlink_scaled_pressure2_t      m_scaledPressure2;
        mavlink_scaled_pressure3_t      m_scaledPressure3;

        mavlink_autopilot_version_t     m_autopilotVersion;


        CraftData* m_craftData;

    private:

        bool m_receivedHeartbeat;
        bool m_receivedGpsRawInt;
        bool m_receivedGps2Raw;
        bool m_receivedGlobalPositionInt;
        bool m_receivedRCChannelsRaw;
        bool m_receivedServoOutputRaw;


};

#endif // CRAFTCONNECTION_H


