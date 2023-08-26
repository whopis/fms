#include "CraftConnection.h"

#include "HFLogger.h"

CraftConnection::CraftConnection()
{
    //ctor
    m_heartbeatCount = 0;
    m_craftData = nullptr;

    m_receivedHeartbeat = false;
    m_receivedGpsRawInt = false;
    m_receivedGps2Raw = false;
    m_receivedGlobalPositionInt = false;
    m_receivedRCChannelsRaw = false;
    m_receivedServoOutputRaw = false;

    strncpy(m_deviceId, "CraftConnection", 25);


}

CraftConnection::~CraftConnection()
{
    //dtor
}


bool CraftConnection::DataStreamValid()
{
    if (m_receivedHeartbeat && m_receivedGpsRawInt && m_receivedGlobalPositionInt && m_receivedRCChannelsRaw && m_receivedServoOutputRaw)
        return true;
    else
        return false;
}


void CraftConnection::AttachCraftData(CraftData* craftData)
{
    m_craftData = craftData;
}



void CraftConnection::HandleHeartbeat(mavlink_message_t* msg)
{
    // Only count heartbeats from gimbal 
    if (msg->compid == 154)
    {
        if (m_craftData != nullptr)
        {
            if (m_craftData->gimbalHeartbeatCount == 0) 
            {
                HFLogger::logMessage("GIMBAL: Heartbeat detected");
            }
            m_craftData->gimbalHeartbeatCount++;
        }
        else
        {
            HFLogger::logMessage("GIMBAL: Heartbeat before craft connected");
        }
        return;
    }
    
    // Ignore any other messages not from flight controller
    if (msg->compid != 1)
    {
        return;
    }

    m_receivedHeartbeat = true;

    mavlink_msg_heartbeat_decode(msg, &m_heartbeat);
    m_heartbeatCount++;
    //HFLogger::logMessage("Heartbeat from %d  %d", msg->sysid, msg->compid);

    if (m_craftData != nullptr)
    {
        if (m_heartbeat.base_mode & 0x80)
        {
            m_craftData->armed = true;
        }
        else
        {
            m_craftData->armed = false;
        }

        m_craftData->custom_mode = m_heartbeat.custom_mode;

        m_craftData->heartbeatCount = m_heartbeatCount;
    }
}


void CraftConnection::HandleSysStatus(mavlink_message_t* msg)
{

    mavlink_sys_status_t sysStatus;
    mavlink_msg_sys_status_decode(msg, &sysStatus);

    if (m_craftData != nullptr)
    {
        HFLogger::logMessage("Voltage - %d  %d", sysStatus.voltage_battery, sysStatus.current_battery);
        m_craftData->batteryCurrent = sysStatus.current_battery;
        m_craftData->batteryVoltage = sysStatus.voltage_battery;
        m_craftData->voltage_set = true;
    }

}

void CraftConnection::HandleParamValue(mavlink_message_t* msg)
{
    mavlink_param_value_t paramValue;
    mavlink_msg_param_value_decode(msg, &paramValue);

    if (m_craftData != nullptr)
    {
        if (strncmp(paramValue.param_id, "STAT_FLTTIME", 16) == 0)
        {
            m_craftData->totalFlightTime = (int) paramValue.param_value;
            HFLogger::logMessage("STAT_FLTTIME: %d", m_craftData->totalFlightTime);
        }
        if (strncmp(paramValue.param_id, "STAT_RUNTIME", 16) == 0)
        {
            m_craftData->totalRunTime = (int) paramValue.param_value;
            HFLogger::logMessage("STAT_RUNTIME: %d", m_craftData->totalRunTime);
        }
        if (strncmp(paramValue.param_id, "STAT_BOOTCNT", 16) == 0)
        {
            m_craftData->totalBootCount = (int) paramValue.param_value;
            HFLogger::logMessage("STAT_BOOTCNT: %d", m_craftData->totalBootCount);
        }
        if (strncmp(paramValue.param_id, "STAT_RESET", 16) == 0)
        {
            m_craftData->timeSinceReset = (int) paramValue.param_value;
            HFLogger::logMessage("STAT_RESET: %d", m_craftData->timeSinceReset);
        }
        
        if (strncmp(paramValue.param_id, "EK2_MAG_RST_ALT", 16) == 0)
        {
            m_craftData->ek2_mag_rst_alt = paramValue.param_value;
            m_craftData->ek2_mag_rst_alt_set = true;
            HFLogger::logMessage("EK2_MAG_RST_ALT: %f", m_craftData->ek2_mag_rst_alt);
        }
        if (strncmp(paramValue.param_id, "EK3_MAG_RST_ALT", 16) == 0)
        {
            m_craftData->ek3_mag_rst_alt = paramValue.param_value;
            m_craftData->ek3_mag_rst_alt_set = true;
            HFLogger::logMessage("EK3_MAG_RST_ALT: %f", m_craftData->ek3_mag_rst_alt);
        }
        if (strncmp(paramValue.param_id, "GPS_AUTO_SWITCH", 16) == 0)
        {
            m_craftData->gps_auto_switch = (int)paramValue.param_value;
            m_craftData->gps_auto_switch_set = true;
            HFLogger::logMessage("GPS_AUTO_SWITCH: %d", m_craftData->ek3_mag_rst_alt);
        }
        
        if (strncmp(paramValue.param_id, "WPNAV_SPEED_UP", 16) == 0)
        {
            m_craftData->wpnav_speed_up = (int)paramValue.param_value;
            m_craftData->wpnav_speed_up_set = true;
            HFLogger::logMessage("WPNAV_SPEED_UP: %d", m_craftData->wpnav_speed_up);
        }
        
        
       
    }
}


void CraftConnection::HandleGpsRawInt(mavlink_message_t* msg)
{

    m_receivedGpsRawInt = true;


    mavlink_gps_raw_int_t gpsRawInt;
    mavlink_msg_gps_raw_int_decode(msg, &gpsRawInt);

    if (m_craftData != nullptr)
    {
        HFLogger::logMessage("CRAFT_GPS_1: h:%d  v:%d  s:%d  f:%d", gpsRawInt.eph, gpsRawInt.epv, gpsRawInt.satellites_visible, gpsRawInt.fix_type);
        m_craftData->gpsHdop = gpsRawInt.eph;
        m_craftData->gpsVdop = gpsRawInt.epv;
        m_craftData->gpsNumberSatellites = gpsRawInt.satellites_visible;
        m_craftData->gpsFixType = gpsRawInt.fix_type;

    }
}



void CraftConnection::HandleGps2Raw(mavlink_message_t* msg)
{

    m_receivedGps2Raw = true;


    mavlink_gps2_raw_t gps2Raw;
    mavlink_msg_gps2_raw_decode(msg, &gps2Raw);

    if (m_craftData != nullptr)
    {
        HFLogger::logMessage("CRAFT_GPS_2: h:%d  v:%d  s:%d  f:%d", gps2Raw.eph, gps2Raw.epv, gps2Raw.satellites_visible, gps2Raw.fix_type);

        m_craftData->gpsHdop2 = gps2Raw.eph;
        m_craftData->gpsVdop2 = gps2Raw.epv;
        m_craftData->gpsNumberSatellites2 = gps2Raw.satellites_visible;
        m_craftData->gpsFixType2 = gps2Raw.fix_type;

    }
}



void CraftConnection::HandleGlobalPositionInt(mavlink_message_t* msg)
{
    if (msg->compid != 1)
    {
        return;
    }

    m_receivedGlobalPositionInt = true;


    mavlink_global_position_int_t globalPositionInt;
    mavlink_msg_global_position_int_decode(msg, &globalPositionInt);

    if (m_craftData != nullptr)
    {
        m_craftData->latitude =  globalPositionInt.lat;
        m_craftData->longitude =  globalPositionInt.lon;
        m_craftData->relative_alt = globalPositionInt.relative_alt;
        m_craftData->heading = globalPositionInt.hdg;
        HFLogger::logMessage("Craft: GlobalPositionInt: lat: %d, lon: %d, rel_alt: %d,  hdg: %d", globalPositionInt.lat, globalPositionInt.lon, globalPositionInt.relative_alt, globalPositionInt.hdg);
    }

}

void CraftConnection::HandleRCChannelsRaw(mavlink_message_t* msg)
{
    m_receivedRCChannelsRaw = true;

    mavlink_rc_channels_raw_t rcChannelsRaw;
    mavlink_msg_rc_channels_raw_decode(msg, &rcChannelsRaw);


    if (m_craftData != nullptr)
    {
        if (rcChannelsRaw.port == 0)
        {
            HFLogger::logMessage("Lower Channels");
            m_craftData->rcChannels[0] = rcChannelsRaw.chan1_raw;
            m_craftData->rcChannels[1] = rcChannelsRaw.chan2_raw;
            m_craftData->rcChannels[2] = rcChannelsRaw.chan3_raw;
            m_craftData->rcChannels[3] = rcChannelsRaw.chan4_raw;
            m_craftData->rcChannels[4] = rcChannelsRaw.chan5_raw;
            m_craftData->rcChannels[5] = rcChannelsRaw.chan6_raw;
            m_craftData->rcChannels[6] = rcChannelsRaw.chan7_raw;
            m_craftData->rcChannels[7] = rcChannelsRaw.chan8_raw;
        }

        if (rcChannelsRaw.port == 1)
        {
            HFLogger::logMessage("Upper Channels %d %d", m_craftData->rcChannels[8], m_craftData->rcChannels[9]);
            m_craftData->rcChannels[8] = rcChannelsRaw.chan1_raw;
            m_craftData->rcChannels[9] = rcChannelsRaw.chan2_raw;
            m_craftData->rcChannels[10] = rcChannelsRaw.chan3_raw;
            m_craftData->rcChannels[11] = rcChannelsRaw.chan4_raw;
            m_craftData->rcChannels[12] = rcChannelsRaw.chan5_raw;
            m_craftData->rcChannels[13] = rcChannelsRaw.chan6_raw;
            m_craftData->rcChannels[14] = rcChannelsRaw.chan7_raw;
            m_craftData->rcChannels[15] = rcChannelsRaw.chan8_raw;
        }
    }

}


void CraftConnection::HandleServoOutputRaw(mavlink_message_t* msg)
{
    m_receivedServoOutputRaw = true;

    mavlink_servo_output_raw_t servoOutputRaw;
    mavlink_msg_servo_output_raw_decode(msg, &servoOutputRaw);


    if (m_craftData != nullptr)
    {
        m_craftData->servoOutputs[0] = servoOutputRaw.servo1_raw;
        m_craftData->servoOutputs[1] = servoOutputRaw.servo2_raw;
        m_craftData->servoOutputs[2] = servoOutputRaw.servo3_raw;
        m_craftData->servoOutputs[3] = servoOutputRaw.servo4_raw;
        m_craftData->servoOutputs[4] = servoOutputRaw.servo5_raw;
        m_craftData->servoOutputs[5] = servoOutputRaw.servo6_raw;
        m_craftData->servoOutputs[6] = servoOutputRaw.servo7_raw;
        m_craftData->servoOutputs[7] = servoOutputRaw.servo8_raw;
    }

}



void CraftConnection::HandleStatusText(mavlink_message_t* msg)
{
    HFLogger::logMessage("CraftConnection - HandleStatusText");

    mavlink_statustext_t statusText;
    mavlink_msg_statustext_decode(msg, &statusText);

    if (m_statusTextQueue != nullptr)
    {
        StatusText* statusTextEntry = new StatusText();
        statusTextEntry->severity = statusText.severity;
        for (int i = 0; i < 50; i++)
        {
            statusTextEntry->text[i] = statusText.text[i];
        }

        m_statusTextQueue->Enqueue(statusTextEntry);
    }
    else
    {
        HFLogger::logMessage("CraftConnection - StatusQueue not set");
    }
}

void CraftConnection::HandleScaledPressure(mavlink_message_t* msg)
{

    mavlink_msg_scaled_pressure_decode(msg, &m_scaledPressure);
    //m_receviedScaledPressureMsg = true;

    HFLogger::logMessage("Craft - Pressure1 %f,  Temperature1 %d", m_scaledPressure.press_abs, m_scaledPressure.temperature);

    if (m_craftData != nullptr)
    {
        m_craftData->pressAbs = m_scaledPressure.press_abs;
        m_craftData->temperature = m_scaledPressure.temperature;
    }
}


void CraftConnection::HandleScaledPressure2(mavlink_message_t* msg)
{

    mavlink_msg_scaled_pressure2_decode(msg, &m_scaledPressure2);
    //m_receviedScaledPressureMsg = true;

    HFLogger::logMessage("Craft - Pressure2 %f,  Temperature2 %d", m_scaledPressure2.press_abs, m_scaledPressure2.temperature);

    if (m_craftData != nullptr)
    {
        m_craftData->pressAbs2 = m_scaledPressure2.press_abs;
        m_craftData->temperature2 = m_scaledPressure2.temperature;
    }
}



void CraftConnection::HandleScaledPressure3(mavlink_message_t* msg)
{

    mavlink_msg_scaled_pressure3_decode(msg, &m_scaledPressure3);
    //m_receviedScaledPressureMsg = true;

    HFLogger::logMessage("Craft - Pressure3 %f,  Temperature3 %d", m_scaledPressure3.press_abs, m_scaledPressure3.temperature);

    if (m_craftData != nullptr)
    {
        m_craftData->pressAbs3 = m_scaledPressure3.press_abs;
        m_craftData->temperature3 = m_scaledPressure3.temperature;
    }
}



void CraftConnection::HandleAutopilotVersion(mavlink_message_t* msg)
{
    mavlink_msg_autopilot_version_decode(msg, &m_autopilotVersion);

    HFLogger::logMessage("Autopilot Version");
    //HFLogger::logMessage("capabilities %d", m_autopilotVersion.capabilities);
    HFLogger::logMessage("flight_sw_version %d", m_autopilotVersion.flight_sw_version);
    HFLogger::logMessage("middleware_sw_version %d", m_autopilotVersion.middleware_sw_version);
    HFLogger::logMessage("os_sw_version %d", m_autopilotVersion.os_sw_version);
    HFLogger::logMessage("board_version %d", m_autopilotVersion.board_version);
    for (int i = 0; i < 8; i++)
    {
        if (m_autopilotVersion.flight_custom_version[i] != 0)
        {
            HFLogger::logMessage("flight_custom_version:%d   %c", i, m_autopilotVersion.flight_custom_version[i]);
        }
        else
        {
            HFLogger::logMessage("flight_custom_version:%d   0", i);
        }
    }

    for (int i = 0; i < 8; i++)
    {
        if (m_autopilotVersion.middleware_custom_version[i] != 0)
        {
            HFLogger::logMessage("middleware_custom_version:%d   %c", i, m_autopilotVersion.middleware_custom_version[i]);
        }
        else
        {
            HFLogger::logMessage("middleware_custom_version:%d   0", i);
        }
    }

    for (int i = 0; i < 8; i++)
    {
        if (m_autopilotVersion.os_custom_version[i] != 0)
        {
            HFLogger::logMessage("os_custom_version:%d   %c", i, m_autopilotVersion.os_custom_version[i]);
        }
        else
        {
            HFLogger::logMessage("os_custom_version:%d   0", i);
        }
    }
//    HFLogger::logMessage("flight_custom_version %d", m_autopilotVersion.flight_custom_version);
//    HFLogger::logMessage("middleware_custom_version %d", m_autopilotVersion.middleware_custom_version);
//    HFLogger::logMessage("os_custom_version %d", m_autopilotVersion.os_custom_version);
    HFLogger::logMessage("vendor_id %d", m_autopilotVersion.vendor_id);
    HFLogger::logMessage("product_id %d", m_autopilotVersion.product_id);
//    HFLogger::logMessage("uid %d", m_autopilotVersion.uid);
/*
    for (int i = 0; i < 18; i++)
    {
        HFLogger::logMessage("uid2:%d   %d", i, m_autopilotVersion.uid2[i]);
    }
*/

    if (m_craftData != nullptr)
    {
    }
}


void CraftConnection::HandleRangefinder(mavlink_message_t* msg)
{
    mavlink_rangefinder_t rangefinder;
    mavlink_msg_rangefinder_decode(msg, &rangefinder);
    //m_receviedScaledPressureMsg = true;

    if (m_craftData != nullptr)
    {
        m_craftData->rangefinderDistance = rangefinder.distance;
        HFLogger::logMessage("RangeFinder: %f", rangefinder.distance);
    }
}


void CraftConnection::HandleVibration(mavlink_message_t* msg)
{
    mavlink_vibration_t vibration;
    mavlink_msg_vibration_decode(msg, &vibration);
    
    if (m_craftData != nullptr)
    {
        m_craftData->setVibrationData(vibration.vibration_x, vibration.vibration_y, vibration.vibration_z, vibration.clipping_0, vibration.clipping_1, vibration.clipping_2);
    }
}

void CraftConnection::RequestDataStream(SourceDeviceType device, uint8_t streamId)
{
    //this->SendRequestDataStream(device, 1, 1, streamId, 1, 1);
    this->SendRequestDataStream(device, 1, 1, streamId, 10, 1); // change to 10Hz telem rate
}


void CraftConnection::OverrideRcChannelsLow(SourceDeviceType device, uint16_t rc1, uint16_t rc2, uint16_t rc3, uint16_t rc4, uint16_t rc5, uint16_t rc6, uint16_t rc7, uint16_t rc8)
{
    this->SendRCOverrides(device, 1, 1, rc1, rc2, rc3, rc4, rc5, rc6, rc7, rc8);
}

void CraftConnection::OverrideRcChannelsHigh(SourceDeviceType device, uint16_t rc1, uint16_t rc2, uint16_t rc3, uint16_t rc4, uint16_t rc5, uint16_t rc6, uint16_t rc7, uint16_t rc8)
{
    this->SendRCOverrides(device, 1, 2, rc1, rc2, rc3, rc4, rc5, rc6, rc7, rc8);
}

