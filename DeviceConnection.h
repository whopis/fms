#ifndef DEVICECONNECTION_H
#define DEVICECONNECTION_H

#include "MAVLinkHandler.h"

#include <iostream>

#include "SourceData.h"

#include "StatusText.h"

using namespace std;

#define CUSTOM_STATUS_IR_PALETTE        101
#define CUSTOM_STATUS_ZOOM_SCALE        251
#define CUSTOM_STATUS_GIMBAL_001        1001
#define CUSTOM_STATUS_GIMBAL_002        1002
#define CUSTOM_STATUS_LOCKER            1101
#define CUSTOM_STATUS_SKYBOX            1200
#define CUSTOM_STATUS_REEL              1400
#define CUSTOM_STATUS_REEL_1            1401
#define CUSTOM_STATUS_REEL_2            1402
#define CUSTOM_STATUS_WIND              2001
#define CUSTOM_STATUS_FLIGHT_LIMITS     2002
#define CUSTOM_STATUS_BGU_STATE         3001
#define CUSTOM_STATUS_FLIGHT_STATUS     4001

class DeviceConnection
{
    public:
        DeviceConnection();
        virtual ~DeviceConnection();

        void AttachMAVLinkHandler(MAVLinkHandler* mavLinkHandler);
        void AttachStatusTextQueue(ConcurrentQueue<StatusText*>* statusTextQueue);


        void Start();
        void Stop();
        bool IsConnected();

        virtual void SendHeartbeat(SourceDeviceType device, uint32_t custom_mode, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint8_t system_status);
        virtual void SendSysStatus(SourceDeviceType device, uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery, uint16_t current_battery, int8_t battery_remaining, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1, uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4);
        virtual void SendRequestDataStream(SourceDeviceType device, uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop);
        virtual void SendRCOverrides(SourceDeviceType device, uint8_t target_system, uint8_t target_component, uint16_t rc1, uint16_t rc2, uint16_t rc3, uint16_t rc4, uint16_t rc5, uint16_t rc6, uint16_t rc7, uint16_t rc8);
        virtual void SendGpsRawInt(SourceDeviceType device, uint64_t time_usec, uint8_t fix_type, uint32_t lat, uint32_t lon, uint32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible);
        virtual void SendGlobalPositionInt(SourceDeviceType device, uint32_t time_boot_ms, uint32_t lat, uint32_t lon, uint32_t alt, uint32_t relative_alt, uint16_t vx, uint16_t vy, uint16_t vz, uint16_t hdg);
        virtual void SendCommandLong(SourceDeviceType device, uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7);
        virtual void SendSetPositionTargetGlobalInt(SourceDeviceType device, uint32_t time_boot_ms, uint8_t targetSystem, uint8_t targetComponent, uint8_t frame, uint16_t mask, int32_t latInt, int32_t lonInt, float altitude, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yawRate);
        virtual void SendMissionItem(SourceDeviceType device, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z);
        virtual void SendRCChannelsRaw(SourceDeviceType device, uint32_t time_boot_ms, uint8_t port, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi);
        virtual void SendStatusText(SourceDeviceType device, uint8_t severity, char* text);
        virtual void SendSystemTime(SourceDeviceType device, uint64_t unix_time_us, uint32_t boot_time_ms);
        virtual void SendAutopilotVersionRequest(SourceDeviceType device, uint8_t targetSystem, uint8_t targetComponent);
        virtual void SendRangefinder(SourceDeviceType device, float distance);
        virtual void SendZoomScale(SourceDeviceType device, int zoomScale);
        virtual void SendIrColorPalette(SourceDeviceType device, int irColorPalette);
        virtual void SendSkyBoxStatusAllConnections(SourceDeviceType device, int rollTopPosition, int scissorLiftPosition, int moveState, int moveTimeRemaining);
        virtual void SendLockerStatusAllConnections(SourceDeviceType device, int moveState);
        virtual void SendGimbalStatus001(SourceDeviceType device, float latitude, float longitude, float relative_alt, float heading, float tiltAngle);
        virtual void SendGimbalStatus002(SourceDeviceType device, float hFoV_current, float vFoV_current, float hFoV_full, float vFoV_full, float magFactor);
        virtual void SendParamValue(SourceDeviceType device, char* param_id, float param_value, MAV_PARAM_TYPE param_type);

        virtual void SendParamSet(SourceDeviceType device, uint8_t target_system, uint8_t target_component, char* param_id, float param_value, uint8_t param_type);

        virtual void SendParamRequestRead(SourceDeviceType device, uint8_t target_system, uint8_t target_component, char* param_id);

        virtual void SendWind(SourceDeviceType device, float windSpeed, float windDirection, float windBearing);
        virtual void SendFlightLimits(SourceDeviceType device, float ceiling, float powerCeiling, bool precisionLandEnabled, bool precisionLandHealthy, int gpsOn);
        
        virtual void SendBguStatus(SourceDeviceType device, float precisionLandStatus, int controllerGpsOn, int bguGpsOn);
        virtual void SendFlightStatus(SourceDeviceType device, bool craftInitiatedLanding);
        
        virtual void SendReelStatus1(SourceDeviceType device, int state, int faultMask, int errorMask, int tetherPos, int tetherWarning, int motorOn);
        virtual void SendReelStatus2(SourceDeviceType device, int voltTension, int volt12, int voltMotor, int currentMotor, int temp, int tempIr);
        
        
        unsigned int GetHeartbeatCount();

        bool IsConnectionActive(int maxSeconds);


        void ClearMissionAck();
        void ClearCommandAck();

        bool MissionAckReceived() { return m_missionAckReceived; }
        uint8_t MissionAckType() { return m_missionAckType; }

        bool CommandAckRecevied() { return m_commandAckReceived; }
        uint16_t CommandAckCommand() { return m_commandAckCommand; }
        uint8_t CommandAckResult() { return m_commandAckResult; }

        bool Master;
        bool NoGroundGps;

    protected:
        virtual void HandleHeartbeat(mavlink_message_t* msg){};           // 000  /   0x00
        virtual void HandleSysStatus(mavlink_message_t* msg){};           // 001  /   0x01
        virtual void HandleParamValue(mavlink_message_t* msg){};          // 022  /   0x16
        virtual void HandleGpsRawInt(mavlink_message_t* msg){};           // 024  /   0x16
        virtual void HandleGps2Raw(mavlink_message_t* msg){};             // 124  /   0x7C
        virtual void HandleGlobalPositionInt(mavlink_message_t* msg){};   // 033  /   0x21
        virtual void HandleRCChannelsRaw(mavlink_message_t* msg){};       // 035  /   0x23
        virtual void HandleServoOutputRaw(mavlink_message_t* msg){};      // 036  /   0x24
        virtual void HandleMissionItem(mavlink_message_t* msg){};         // 039  /   0x27
        virtual void HandleMissionAck(mavlink_message_t* msg);            // 047  /   0x2F
        virtual void HandleManualControl(mavlink_message_t* msg){};       // 069  /   0x45
        virtual void HandleCommandLong(mavlink_message_t* msg){};         // 076  /   0x4C
        virtual void HandleCommandAck(mavlink_message_t* msg);            // 077  /   0x4D
        virtual void HandleStatusText(mavlink_message_t* msg){};          // 253  /   0xFD
        virtual void HandleScaledPressure(mavlink_message_t* msg){};      // 029  /   0x1D
        virtual void HandleScaledPressure2(mavlink_message_t* msg){};
        virtual void HandleScaledPressure3(mavlink_message_t* msg){};     // 143  /   0x8F
        virtual void HandleAutopilotVersion(mavlink_message_t* msg){};    // 148  /   0x94
        virtual void HandleRcChannelsOverride(mavlink_message_t* msg){};  // 070  /   0x46
        virtual void HandleRangefinder(mavlink_message_t* msg){};         // 173  /   0xAD
        virtual void HandleVibration(mavlink_message_t* msg){};           // 241  /   0xF1



        const char* DeviceId() { return m_deviceId; };


        MAVLinkHandler* m_mavLinkHandler;

        ConcurrentQueue<mavlink_message_t*> m_messageQueue;

        ConcurrentQueue<StatusText*>* m_statusTextQueue;

        std::thread m_messageThread;

        void MessageHandler();

        bool m_isConnected;

        unsigned int m_heartbeatCount;

        uint8_t m_missionAckType;
        bool m_missionAckReceived;

        uint16_t m_commandAckCommand;
        uint8_t m_commandAckResult;
        bool m_commandAckReceived;

        char m_deviceId[25];

    private:


};

#endif // CONNECTION_H
