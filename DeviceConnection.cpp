#include "DeviceConnection.h"

#include "HFLogger.h"


DeviceConnection::DeviceConnection()
{
    //ctor
    m_heartbeatCount = 0;
    m_mavLinkHandler = nullptr;
    m_statusTextQueue = nullptr;

    Master = false;
    NoGroundGps = false;
}

DeviceConnection::~DeviceConnection()
{
    //dtor
    delete m_mavLinkHandler;
}


void DeviceConnection::AttachMAVLinkHandler(MAVLinkHandler* mavLinkHandler)
{
    m_mavLinkHandler = mavLinkHandler;
    m_mavLinkHandler->AttachMessageQueue(&m_messageQueue);
}

void DeviceConnection::AttachStatusTextQueue(ConcurrentQueue<StatusText*>* statusTextQueue)
{
    m_statusTextQueue = statusTextQueue;
}

void DeviceConnection::Start()
{
    m_isConnected = true;
    m_mavLinkHandler->Start();

    // Begin a thread to handle incoming MAVLink messages
    m_messageThread = std::thread(&DeviceConnection::MessageHandler, this);

}

void DeviceConnection::Stop()
{
    m_mavLinkHandler->Stop();
    m_messageThread.join();
}


bool DeviceConnection::IsConnected()
{
    return m_isConnected;
}


bool DeviceConnection::IsConnectionActive(int maxSeconds)
{
    bool active = false;

    if (m_mavLinkHandler != nullptr)
    {
        active = m_mavLinkHandler->IsConnectionActive(maxSeconds);
    }

    return active;

}


void DeviceConnection::SendHeartbeat(SourceDeviceType device, uint32_t custom_mode, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint8_t system_status)
{
    mavlink_heartbeat_t heartbeat;
    memset(&heartbeat, 0, sizeof(mavlink_heartbeat_t));
    heartbeat.custom_mode = custom_mode;
    heartbeat.type = type;
    heartbeat.autopilot = autopilot;
    heartbeat.base_mode = base_mode;
    heartbeat.system_status = system_status;

    m_mavLinkHandler->SendMessage_Heartbeat(device, &heartbeat);


    //uint8_t* buf = new uint8_t[MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_HEARTBEAT_LEN];
    //mavlink_msg_to_send_buffer(buf, &msg);

    //m_mavLinkHandler->SendMessage(buf);
}



void DeviceConnection::SendSysStatus(SourceDeviceType device, uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery, uint16_t current_battery, int8_t battery_remaining, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1, uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4)
{
    mavlink_sys_status_t sysStatus;
    memset(&sysStatus, 0, sizeof(mavlink_sys_status_t));
    sysStatus.onboard_control_sensors_present = onboard_control_sensors_present;
    sysStatus.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    sysStatus.onboard_control_sensors_health = onboard_control_sensors_health;
    sysStatus.load = load;
    sysStatus.voltage_battery = voltage_battery;
    sysStatus.current_battery = current_battery;
    sysStatus.battery_remaining = battery_remaining;
    sysStatus.drop_rate_comm = drop_rate_comm;
    sysStatus.errors_comm = errors_comm;
    sysStatus.errors_count1 = errors_count1;
    sysStatus.errors_count2 = errors_count2;
    sysStatus.errors_count3 = errors_count3;
    sysStatus.errors_count4 = errors_count4;

    m_mavLinkHandler->SendMessage_SysStatus(device, &sysStatus);

}


void DeviceConnection::SendRequestDataStream(SourceDeviceType device, uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)
{
    mavlink_request_data_stream_t requestDataStream;
    memset(&requestDataStream, 0, sizeof(mavlink_request_data_stream_t));
    requestDataStream.target_system = target_system;
    requestDataStream.target_component = target_component;
    requestDataStream.req_stream_id = req_stream_id;
    requestDataStream.req_message_rate = req_message_rate;
    requestDataStream.start_stop = start_stop;

    m_mavLinkHandler->SendMessage_RequestDataStream(device, &requestDataStream);

}

void DeviceConnection::SendRCOverrides(SourceDeviceType device, uint8_t target_system, uint8_t target_component, uint16_t rc1, uint16_t rc2, uint16_t rc3, uint16_t rc4, uint16_t rc5, uint16_t rc6, uint16_t rc7, uint16_t rc8)
{
    mavlink_rc_channels_override_t rcChannelsOverride;
    memset(&rcChannelsOverride, 0, sizeof(mavlink_rc_channels_override_t));
    rcChannelsOverride.target_system = target_system;
    rcChannelsOverride.target_component = target_component;
    rcChannelsOverride.chan1_raw = rc1;
    rcChannelsOverride.chan2_raw = rc2;
    rcChannelsOverride.chan3_raw = rc3;
    rcChannelsOverride.chan4_raw = rc4;
    rcChannelsOverride.chan5_raw = rc5;
    rcChannelsOverride.chan6_raw = rc6;
    rcChannelsOverride.chan7_raw = rc7;
    rcChannelsOverride.chan8_raw = rc8;

    m_mavLinkHandler->SendMessage_RCChannelsOverride(device, &rcChannelsOverride);

}


void DeviceConnection::SendGpsRawInt(SourceDeviceType device, uint64_t time_usec, uint8_t fix_type, uint32_t lat, uint32_t lon, uint32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible)
{
    mavlink_gps_raw_int_t gpsRawInt;
    memset(&gpsRawInt, 0, sizeof(mavlink_gps_raw_int_t));
    gpsRawInt.time_usec = time_usec;
    gpsRawInt.fix_type = fix_type;
    gpsRawInt.lat = lat;
    gpsRawInt.lon = lon;
    gpsRawInt.alt = alt;
    gpsRawInt.eph = eph;
    gpsRawInt.epv = epv;
    gpsRawInt.vel = vel;
    gpsRawInt.cog = cog;
    gpsRawInt.satellites_visible = satellites_visible;

    m_mavLinkHandler->SendMessage_GpsRawInt(device, &gpsRawInt);

}


void DeviceConnection::SendGlobalPositionInt(SourceDeviceType device, uint32_t time_boot_ms, uint32_t lat, uint32_t lon, uint32_t alt, uint32_t relative_alt, uint16_t vx, uint16_t vy, uint16_t vz, uint16_t hdg)
{

    mavlink_global_position_int_t globalPositionInt;
    memset(&globalPositionInt, 0, sizeof(mavlink_global_position_int_t));
    globalPositionInt.time_boot_ms = time_boot_ms;
    globalPositionInt.lat = lat;
    globalPositionInt.lon = lon;
    globalPositionInt.alt = alt;
    globalPositionInt.relative_alt = relative_alt;
    globalPositionInt.vx = vx;
    globalPositionInt.vy = vy;
    globalPositionInt.vz = vz;
    globalPositionInt.hdg = hdg;

    m_mavLinkHandler->SendMessage_GlobalPositionInt(device, &globalPositionInt);
}



void DeviceConnection::SendRCChannelsRaw(SourceDeviceType device, uint32_t time_boot_ms, uint8_t port, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi)
{
    mavlink_rc_channels_raw_t rcChannelsRaw;
    memset(&rcChannelsRaw, 0, sizeof(mavlink_rc_channels_raw_t));
    rcChannelsRaw.time_boot_ms = time_boot_ms;
    rcChannelsRaw.port = port;
    rcChannelsRaw.chan1_raw = chan1_raw;
    rcChannelsRaw.chan2_raw = chan2_raw;
    rcChannelsRaw.chan3_raw = chan3_raw;
    rcChannelsRaw.chan4_raw = chan4_raw;
    rcChannelsRaw.chan5_raw = chan5_raw;
    rcChannelsRaw.chan6_raw = chan6_raw;
    rcChannelsRaw.chan7_raw = chan7_raw;
    rcChannelsRaw.chan8_raw = chan8_raw;
    rcChannelsRaw.rssi = rssi;

    m_mavLinkHandler->SendMessage_RCChannelsRaw(device, &rcChannelsRaw);
}

void DeviceConnection::SendStatusText(SourceDeviceType device, uint8_t severity, char* text)
{
    mavlink_statustext_t statusText;
    memset(&statusText, 0, sizeof(mavlink_statustext_t));
    statusText.severity = severity;
    for (int i = 0; i < 50; i++)
    {
        statusText.text[i] = text[i];
    }

    m_mavLinkHandler->SendMessage_StatusText(device, &statusText);
}


void DeviceConnection::SendParamSet(SourceDeviceType device, uint8_t target_system, uint8_t target_component, char* param_id, float param_value, uint8_t param_type)
{
    mavlink_param_set_t paramSet;
    memset(&paramSet, 0, sizeof(mavlink_param_set_t));

    paramSet.target_system = target_system;
    paramSet.target_component = target_component;
    paramSet.param_value = param_value;
    paramSet.param_type = param_type;
    for (int i = 0; i < 16; i++)
    {
        paramSet.param_id[i] = param_id[i];
    }

    m_mavLinkHandler->SendMessage_ParamSet(device, &paramSet);
}


void DeviceConnection::SendParamRequestRead(SourceDeviceType device, uint8_t target_system, uint8_t target_component, char* param_id)
{
    mavlink_param_request_read_t paramRequest;
    memset(&paramRequest, 0, sizeof(mavlink_param_request_read_t));

    paramRequest.target_system = target_system;
    paramRequest.target_component = target_component;
    paramRequest.param_index = -1;
    for (int i = 0; i < 16; i++)
    {
        paramRequest.param_id[i] = param_id[i];
    }

    m_mavLinkHandler->SendMessage_ParamRequestRead(device, &paramRequest);
}



void DeviceConnection::SendParamValue(SourceDeviceType device, char* param_id, float param_value, MAV_PARAM_TYPE param_type)
{
    mavlink_param_value_t paramValue;
    memset(&paramValue, 0, sizeof(mavlink_param_value_t));

    paramValue.param_index = -1;
    for (int i = 0; i < 16; i++)
    {
        paramValue.param_id[i] = param_id[i];
    }
    paramValue.param_value = param_value;
    paramValue.param_type = param_type;

    m_mavLinkHandler->SendMessage_ParamValue(device, &paramValue);
}



void DeviceConnection::SendSetPositionTargetGlobalInt(SourceDeviceType device, uint32_t time_boot_ms, uint8_t targetSystem, uint8_t targetComponent, uint8_t frame, uint16_t mask, int32_t latInt, int32_t lonInt, float altitude, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yawRate)
{
    mavlink_set_position_target_global_int_t positionTarget;
    memset(&positionTarget, 0, sizeof(mavlink_set_position_target_global_int_t));
    positionTarget.target_system = targetSystem;
    positionTarget.target_component = targetComponent;
    positionTarget.time_boot_ms = time_boot_ms;
    positionTarget.lat_int = latInt;
    positionTarget.lon_int = lonInt;
    positionTarget.alt = altitude;
    positionTarget.vx = vx;
    positionTarget.vy = vy;
    positionTarget.vz = vz;
    positionTarget.afx = afx;
    positionTarget.afy = afy;
    positionTarget.afz = afz;
    positionTarget.yaw_rate = yawRate;
    positionTarget.yaw = yaw;
    positionTarget.type_mask = mask;
    positionTarget.coordinate_frame = frame;


    m_mavLinkHandler->SendMessage_SetPositionTargetGlobalInt(device, &positionTarget);
}





void DeviceConnection::SendCommandLong(SourceDeviceType device, uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{

    mavlink_command_long_t commandLong;
    memset(&commandLong, 0, sizeof(mavlink_command_long_t));
    commandLong.target_system = target_system;
    commandLong.target_component = target_component;
    commandLong.command = command;
    commandLong.confirmation = confirmation;
    commandLong.param1 = param1;
    commandLong.param2 = param2;
    commandLong.param3 = param3;
    commandLong.param4 = param4;
    commandLong.param5 = param5;
    commandLong.param6 = param6;
    commandLong.param7 = param7;

    //HFLogger::logMessage("SendCommandLong %d : p1=%f : p2=%f : p3=%f : p4=%f : p5=%f : p6=%f : p7=%f", command, param1, param2, param3, param4, param5, param6, param7);
    m_mavLinkHandler->SendMessage_CommandLong(device, &commandLong);
}


void DeviceConnection::SendMissionItem(SourceDeviceType device, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z)
{
    mavlink_mission_item_t missionItem;
    memset(&missionItem, 0, sizeof(mavlink_mission_item_t));
    missionItem.target_system = target_system;
    missionItem.target_component = target_component;
    missionItem.seq = seq;
    missionItem.frame = frame;
    missionItem.command = command;
    missionItem.current = current;
    missionItem.autocontinue = autocontinue;
    missionItem.param1 = param1;
    missionItem.param2 = param2;
    missionItem.param3 = param3;
    missionItem.param4 = param4;
    missionItem.x = x;
    missionItem.y = y;
    missionItem.z = z;

    HFLogger::logMessage("SendMissionItem %d", command);
    m_mavLinkHandler->SendMessage_MissionItem(device, &missionItem);
}

void DeviceConnection::SendSystemTime(SourceDeviceType device, uint64_t unix_time_us, uint32_t boot_time_ms)
{
    mavlink_system_time_t systemTime;
    memset(&systemTime, 0, sizeof(mavlink_system_time_t));
    systemTime.time_unix_usec = unix_time_us;
    systemTime.time_boot_ms = boot_time_ms;

    HFLogger::logMessage("SendSystemTime");
    m_mavLinkHandler->SendMessage_SystemTime(device, &systemTime);
}


void DeviceConnection::SendAutopilotVersionRequest(SourceDeviceType device, uint8_t targetSystem, uint8_t targetComponent)
{
    mavlink_autopilot_version_request_t autopilotVersionRequest;
    memset(&autopilotVersionRequest, 0, sizeof(mavlink_autopilot_version_request_t));
    autopilotVersionRequest.target_system = targetSystem;
    autopilotVersionRequest.target_component = targetComponent;

    HFLogger::logMessage("SendAutopilotVersionRequest");
    m_mavLinkHandler->SendMessage_AutopilotVersionRequest(device, &autopilotVersionRequest);
}


void DeviceConnection::SendRangefinder(SourceDeviceType device, float distance)
{
    mavlink_rangefinder_t rangefinder;
    memset(&rangefinder, 0, sizeof(mavlink_rangefinder_t));
    rangefinder.distance = distance;
    rangefinder.voltage = 0.0f;

    HFLogger::logMessage("SendRangefinder");
    m_mavLinkHandler->SendMessage_Rangefinder(device, &rangefinder);
}



void DeviceConnection::SendZoomScale(SourceDeviceType device, int zoomScale)
{
    SendCommandLong(device, 1, 1, MAV_CMD_USER_4, 0, CUSTOM_STATUS_ZOOM_SCALE, zoomScale, 0, 0, 0, 0, 0);
}


void DeviceConnection::SendIrColorPalette(SourceDeviceType device, int irColorPalette)
{
    SendCommandLong(device, 1, 1, MAV_CMD_USER_4, 0, CUSTOM_STATUS_IR_PALETTE, irColorPalette, 0, 0, 0, 0, 0);
}


void DeviceConnection::SendSkyBoxStatusAllConnections(SourceDeviceType device, int rollTopPosition, int scissorLiftPosition, int moveState, int moveTimeRemaining)
{
    SendCommandLong(device, 1, 1, MAV_CMD_USER_4, 0, CUSTOM_STATUS_SKYBOX, rollTopPosition, scissorLiftPosition, 0, 0, moveState, moveTimeRemaining);
}

void DeviceConnection::SendLockerStatusAllConnections(SourceDeviceType device, int moveState)
{
    //SendCommandLong(device, 1, 1, MAV_CMD_USER_4, 0, CUSTOM_STATUS_LOCKER, 0, 0, 0, 0, moveState, 0);
    // moveState now includes new values that an existing GUI (or custom SDK controller doesn't know about so preserve old behavior:
    // send new moveState as a new parameter;
    // send the acceptable old style move state in the origial parameter location
    int moveStateOldStyle = 1;
    switch (moveState)
    {
        case 0:   // nominal unlocked position UNLOCK_A1B1:
        case 11:  // non-optimal unlocked position UNLOCK_A1B2:
        case 12:  // non-optimal unlocked position UNLOCK_A2B1:
        case 13:  // non-optimal unlocked position UNLOCK_A2B2:
            moveStateOldStyle = 0;
            break;
        case 1:  // moving (A mid : B mid )
        case 31:    // mid  (A locked : B mid)
        case 32:    // mid  (A unlocked primary : B mid)
        case 33:    // mid  (A unlocked seconadry : B mid)
        case 34:    // mid  (A mid : B locked)
        case 35:    // mid  (A mid : B unlocked primary)
        case 36:    // mid  (A mid : B unlocked secondary)
           moveStateOldStyle = 1;
            break;
        case 2:     // nominal locked position (A locked : B locked )
        case 21:    // locked  (A locked : B unlocked primary)
        case 22:    // locked  (A locked : B unlocked secondary)
        case 23:    // locked  (A unlocked primary: B locked)
        case 24:    // locked  (A unlocked secondary: B locked)
           moveStateOldStyle = 2;
            break;
        //case 3:  // UNLOCKED_FAULT (superceded by individual non-optimal unlock positions (11, 12, 13)
        //case 4:  // ERROR - servos done moving but not at desired position (failed to lock a servo)
        default:
            // Treat these new and any unknown state as 'moving'
            moveStateOldStyle = 1;
            break;
    }
    SendCommandLong(device, 1, 1, MAV_CMD_USER_4, 0, CUSTOM_STATUS_LOCKER, 0, 0, 0, 0, moveStateOldStyle, moveState);
}


void DeviceConnection::SendGimbalStatus001(SourceDeviceType device, float latitude, float longitude, float relative_alt, float heading, float tiltAngle)
{
    SendCommandLong(device, 1, 1, MAV_CMD_USER_4, 0, CUSTOM_STATUS_GIMBAL_001, latitude, longitude, relative_alt, heading, tiltAngle, 0);
}

void DeviceConnection::SendGimbalStatus002(SourceDeviceType device, float hFoV_current, float vFoV_current, float hFoV_full, float vFoV_full, float magFactor)
{
    SendCommandLong(device, 1, 1, MAV_CMD_USER_4, 0, CUSTOM_STATUS_GIMBAL_002, hFoV_current, vFoV_current, hFoV_full, vFoV_full, magFactor, 0);
}



void DeviceConnection::SendWind(SourceDeviceType device, float windSpeed, float windDirection, float windBearing)
{
    SendCommandLong(device, 1, 1, MAV_CMD_USER_4, 0, CUSTOM_STATUS_WIND, windSpeed, windDirection, windBearing, 0, 0, 0);
}


void DeviceConnection::SendFlightLimits(SourceDeviceType device, float ceiling, float powerCeiling, bool precisionLandEnabled, bool precisionLandHealthy, int gpsOn)
{
    int param4_plEnabled = precisionLandEnabled ? 1 : 0;
    int param5_plFailure = precisionLandHealthy ? 0 : 1;
    
    HFLogger::logMessage("PL_STATE: %d %d", param4_plEnabled, param5_plFailure);
    
    SendCommandLong(device, 1, 1, MAV_CMD_USER_4, 0, CUSTOM_STATUS_FLIGHT_LIMITS, ceiling, powerCeiling, param4_plEnabled, param5_plFailure, gpsOn, 0);
}


void DeviceConnection::SendBguStatus(SourceDeviceType device, float precisionLandStatus, int controllerGpsOn, int bguGpsOn)
{
        
    SendCommandLong(device, 1, 1, MAV_CMD_USER_4, 0, CUSTOM_STATUS_BGU_STATE, precisionLandStatus, controllerGpsOn, 100, bguGpsOn, 0, 0);
}


void DeviceConnection::SendFlightStatus(SourceDeviceType device, bool craftInitiatedLanding)
{
    int param2 = craftInitiatedLanding ? 1 : 0;
    
    SendCommandLong(device, 1, 1, MAV_CMD_USER_4, 0, CUSTOM_STATUS_FLIGHT_STATUS, param2, 0, 0, 0, 0, 0);
}





void DeviceConnection::SendReelStatus1(SourceDeviceType device, int state, int faultMask, int errorMask, int tetherPos, int tetherWarning, int motorOn)
{    
    SendCommandLong(device, 1, 1, MAV_CMD_USER_4, 0, CUSTOM_STATUS_REEL_1, state, faultMask, errorMask, tetherPos, tetherWarning, motorOn);    
}

void DeviceConnection::SendReelStatus2(SourceDeviceType device, int voltTension, int volt12, int voltMotor, int currentMotor, int temp, int tempIr)
{
    SendCommandLong(device, 1, 1, MAV_CMD_USER_4, 0, CUSTOM_STATUS_REEL_2, voltTension, volt12, voltMotor, currentMotor, temp, tempIr);    
}

        


unsigned int DeviceConnection::GetHeartbeatCount()
{
    return m_heartbeatCount;
}


void DeviceConnection::MessageHandler()
{

    bool runMessageHandler = true;
    while (runMessageHandler == true)
    {
        mavlink_message_t* msg = nullptr;
        if (m_messageQueue.Dequeue(msg) == true)
        {
            //HFLogger::logMessage("MESSAGE:  S:%03d   C:%03d    M:%03d   on %s", msg->sysid, msg->compid, msg->msgid, DeviceId());
            // Unpack message
            switch (msg->msgid)
            {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    HandleHeartbeat(msg);
                    break;
                case MAVLINK_MSG_ID_SYS_STATUS:
                    HandleSysStatus(msg);
                    break;
                    case MAVLINK_MSG_ID_PARAM_VALUE:
                    HandleParamValue(msg);
                    break;
                case MAVLINK_MSG_ID_GPS_RAW_INT:
                    HandleGpsRawInt(msg);
                    break;
                case MAVLINK_MSG_ID_GPS2_RAW:
                    HandleGps2Raw(msg);
                    break;
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                    HandleGlobalPositionInt(msg);
                    break;
                case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                    HandleRCChannelsRaw(msg);
                    break;
                case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
                    HandleServoOutputRaw(msg);
                    break;
                case MAVLINK_MSG_ID_MISSION_ITEM:
                    HandleMissionItem(msg);
                    break;
                case MAVLINK_MSG_ID_MISSION_ACK:
                    HandleMissionAck(msg);
                    break;
                case MAVLINK_MSG_ID_MANUAL_CONTROL:
                    HandleManualControl(msg);
                    break;
                case MAVLINK_MSG_ID_COMMAND_LONG:
                    HandleCommandLong(msg);
                    break;
                case MAVLINK_MSG_ID_COMMAND_ACK:
                    HandleCommandAck(msg);
                    break;
                case MAVLINK_MSG_ID_STATUSTEXT:
                    HandleStatusText(msg);
                    break;
                case MAVLINK_MSG_ID_SCALED_PRESSURE:
                    HandleScaledPressure(msg);
                    break;
                case MAVLINK_MSG_ID_SCALED_PRESSURE2:
                    HandleScaledPressure2(msg);
                    break;
                case MAVLINK_MSG_ID_SCALED_PRESSURE3:
                    HandleScaledPressure3(msg);
                    break;
                case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
                    HandleAutopilotVersion(msg);
                    break;
                case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
                    HandleRcChannelsOverride(msg);
                    break;
                case MAVLINK_MSG_ID_RANGEFINDER:
                    HandleRangefinder(msg);
                    break;
                default:
                    break;
            }

            delete msg;
        }
        else
        {
            // The underlying queue may have been aborted
            if (m_mavLinkHandler->IsConnected() == false)
            {
                runMessageHandler = false;
                m_messageQueue.FlushQueue();
                m_isConnected = false;
            }
        }

    }
}



void DeviceConnection::HandleMissionAck(mavlink_message_t* msg)
{
    mavlink_mission_ack_t missionAck;
    mavlink_msg_mission_ack_decode(msg, &missionAck);

    m_missionAckType = missionAck.type;
    m_missionAckReceived = true;

    HFLogger::logMessage("MissionAck - %d", missionAck.type);
}

void DeviceConnection::HandleCommandAck(mavlink_message_t* msg)
{
    mavlink_command_ack_t commandAck;
    mavlink_msg_command_ack_decode(msg, &commandAck);

    m_commandAckCommand = commandAck.command;
    m_commandAckResult = commandAck.result;
    m_commandAckReceived = true;

    HFLogger::logMessage("CommandAck - %d : %d", commandAck.command, commandAck.result);
}


void DeviceConnection::ClearMissionAck()
{
    m_missionAckType = 0;
    m_missionAckReceived = false;
}


void DeviceConnection::ClearCommandAck()
{
    m_commandAckCommand = 0;
    m_commandAckResult = 0;
    m_commandAckReceived = false;
}




