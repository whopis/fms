#ifndef MAVLINKHANDLER_H
#define MAVLINKHANDLER_H

#include "SerialDataHandler.h"


#include <thread>

#include "mavlink/ardupilotmega/mavlink.h"

#include "SourceData.h"


#define PACKET_STATE_SYNC       0
#define PACKET_STATE_LENGTH     1
#define PACKET_STATE_REMAINDER  2

class MAVLinkHandler
{
    public:
        MAVLinkHandler(SerialDataHandler* serialDataHandler);
        virtual ~MAVLinkHandler();

        void AttachMessageQueue(ConcurrentQueue<mavlink_message_t*>* messageQueue);

        void Start();
        void Stop();
        bool IsConnected();

        void SendMessage(mavlink_message_t* msg);


        void SendMessage_Heartbeat(SourceDeviceType device, mavlink_heartbeat_t* msgHeartbeat);
        void SendMessage_SysStatus(SourceDeviceType device, mavlink_sys_status_t* msgSysStatus);
        void SendMessage_RequestDataStream(SourceDeviceType device, mavlink_request_data_stream_t* msgRequestDataStream);
        void SendMessage_RCChannelsOverride(SourceDeviceType device, mavlink_rc_channels_override_t* msgRCChannelsOverride);
        void SendMessage_GpsRawInt(SourceDeviceType device, mavlink_gps_raw_int_t* msgGpsRawInt);
        void SendMessage_GlobalPositionInt(SourceDeviceType device, mavlink_global_position_int_t* msgGlobalPositionInt);
        void SendMessage_CommandLong(SourceDeviceType device, mavlink_command_long_t* msgCommandLong);
        void SendMessage_SetPositionTargetGlobalInt(SourceDeviceType device, mavlink_set_position_target_global_int_t* msgSetPositionTargetGlobalInt);
        void SendMessage_MissionItem(SourceDeviceType device, mavlink_mission_item_t* msgMissionItem);
        void SendMessage_RCChannelsRaw(SourceDeviceType device, mavlink_rc_channels_raw_t* rcChannelsRaw);
        void SendMessage_StatusText(SourceDeviceType device, mavlink_statustext_t* statusText);
        void SendMessage_SystemTime(SourceDeviceType device, mavlink_system_time_t* systemTime);
        void SendMessage_AutopilotVersionRequest(SourceDeviceType device, mavlink_autopilot_version_request_t* autopilotVersionRequest);
        void SendMessage_Rangefinder(SourceDeviceType device, mavlink_rangefinder_t* rangefinder);

        void SendMessage_ParamSet(SourceDeviceType device, mavlink_param_set_t* paramSet);
        void SendMessage_ParamRequestRead(SourceDeviceType device, mavlink_param_request_read_t* paramRequest);
        void SendMessage_ParamValue(SourceDeviceType device, mavlink_param_value_t* paramValue);

        bool IsConnectionActive(int maxSeconds);


    protected:
    private:
        SerialDataHandler* m_serialDataHandler;

        ConcurrentQueue<std::vector<uint8_t>*> m_incomingDataQueue;

        ConcurrentQueue<mavlink_message_t*>* m_messageQueue;


        std::thread m_packetThread;

        int packet_state;
        int packet_totalLength;
        int packet_payloadLength;
        int packet_currentPosition;
        uint8_t* packet_buffer;

        void PacketHandler();
        void HandleFullPacket(uint8_t* buffer);

        uint8_t*  ParseByte(uint8_t dataByte);

        uint8_t m_messageCRCs[256] = MAVLINK_MESSAGE_CRCS;

        bool m_isConnected;

        int m_goodPacketCount;
        int m_badPacketCount;
        int m_totalPacketCount;

};

#endif // MAVLINKHANDLER_H
