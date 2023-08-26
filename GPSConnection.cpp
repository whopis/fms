#include "GPSConnection.h"

#include "HFLogger.h"

GPSConnection::GPSConnection()
{
    //ctor

    m_heartbeatCount = 0;
    m_receivedGPSRawMsg = false;
    m_receivedGlobalPositionMsg = false;
    m_receivedScaledPressureMsg = false;
    m_receivedScaledPressure2Msg = false;
    m_gpsData = nullptr;

    strncpy(m_deviceId, "GroundGPSConnection", 25);

}

GPSConnection::~GPSConnection()
{
    //dtor
}

bool GPSConnection::DataStreamValid()
{
    bool valid = false;
    if (m_heartbeatCount > 5)
    {
        if ((m_receivedGPSRawMsg == true) && (m_receivedGlobalPositionMsg == true))
        {
            valid = true;
        }
    }

    return valid;
}

bool GPSConnection::PressureReceived()
{
    return m_receivedScaledPressureMsg;
}

bool GPSConnection::Pressure2Received()
{
    return m_receivedScaledPressure2Msg;
}

void GPSConnection::RequestDataStream(SourceDeviceType device, uint8_t streamId)
{
    this->SendRequestDataStream(device, 1, 1, streamId, 1, 1);
}


void GPSConnection::AttachGpsData(GPSData* gpsData)
{
    m_gpsData = gpsData;
}

void GPSConnection::HandleHeartbeat(mavlink_message_t* msg)
{
    mavlink_msg_heartbeat_decode(msg, &m_heartbeat);
    m_heartbeatCount++;

}


void GPSConnection::HandleGpsRawInt(mavlink_message_t* msg)
{

    mavlink_msg_gps_raw_int_decode(msg, &m_gpsRawInt);
    m_receivedGPSRawMsg = true;

    if (m_gpsData != nullptr)
    {
        m_gpsData->gpsHdop = m_gpsRawInt.eph;
        m_gpsData->gpsVdop = m_gpsRawInt.epv;
        m_gpsData->gpsFixType = m_gpsRawInt.fix_type;
        m_gpsData->gpsNumberSatellites = m_gpsRawInt.satellites_visible;
    }

}

void GPSConnection::HandleGlobalPositionInt(mavlink_message_t* msg)
{
    mavlink_msg_global_position_int_decode(msg, &m_globalPositionInt);
    m_receivedGlobalPositionMsg = true;

    if (m_gpsData != nullptr)
    {
        m_gpsData->latitude = m_globalPositionInt.lat;
        m_gpsData->longitude = m_globalPositionInt.lon;
        m_gpsData->relative_alt = m_globalPositionInt.alt;
        m_gpsData->heading = m_globalPositionInt.hdg;
        HFLogger::logMessage("Ground: GlobalPositionInt: lat: %d, lon: %d, rel_alt: %d,  hdg: %d", m_globalPositionInt.lat, m_globalPositionInt.lon, m_globalPositionInt.relative_alt, m_globalPositionInt.hdg);
    }

}

void GPSConnection::HandleScaledPressure(mavlink_message_t* msg)
{
    mavlink_msg_scaled_pressure_decode(msg, &m_scaledPressure);
    m_receivedScaledPressureMsg = true;

    HFLogger::logMessage("Ground: ScaledPressure: press: %f  temp: %d", m_scaledPressure.press_abs, m_scaledPressure.temperature);

    if (m_gpsData != nullptr)
    {
        m_gpsData->pressAbs = m_scaledPressure.press_abs;
        m_gpsData->temperature = m_scaledPressure.temperature;
    }
}


void GPSConnection::HandleScaledPressure2(mavlink_message_t* msg)
{
    mavlink_msg_scaled_pressure2_decode(msg, &m_scaledPressure2);
    m_receivedScaledPressure2Msg = true;

    HFLogger::logMessage("Ground: ScaledPressure2: press: %f  temp: %d", m_scaledPressure2.press_abs, m_scaledPressure2.temperature);

    if (m_gpsData != nullptr)
    {
        m_gpsData->pressAbs2 = m_scaledPressure2.press_abs;
        m_gpsData->temperature2 = m_scaledPressure2.temperature;
    }
}


