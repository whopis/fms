#include "BaroConnection.h"

#include "HFLogger.h"

BaroConnection::BaroConnection()
{
    //ctor

    m_heartbeatCount = 0;
    m_receivedScaledPressureMsg = false;
    m_baroData = nullptr;

    strncpy(m_deviceId, "BaroConnection", 25);

}

BaroConnection::~BaroConnection()
{
    //dtor
}

bool BaroConnection::DataStreamValid()
{
    bool valid = false;
    if (m_heartbeatCount > 5)
    {
        if (m_receivedScaledPressureMsg == true)
        {
            valid = true;
        }
    }

    return valid;
}

bool BaroConnection::PressureReceived()
{
    return m_receivedScaledPressureMsg;
}

void BaroConnection::RequestDataStream(SourceDeviceType device, uint8_t streamId)
{
    this->SendRequestDataStream(device, 1, 1, streamId, 1, 1);
}


void BaroConnection::AttachBaroData(BaroData* baroData)
{
    m_baroData = baroData;
}

void BaroConnection::HandleHeartbeat(mavlink_message_t* msg)
{
    mavlink_msg_heartbeat_decode(msg, &m_heartbeat);
    m_heartbeatCount++;

}



void BaroConnection::HandleScaledPressure(mavlink_message_t* msg)
{
    mavlink_msg_scaled_pressure_decode(msg, &m_scaledPressure);

    HFLogger::logMessage("Baro Data Raw --- p: %f   t: %d", m_scaledPressure.press_abs, m_scaledPressure.temperature);
    m_receivedScaledPressureMsg = true;
    if (m_baroData != nullptr)
    {
        //m_baroData->pressAbs = m_scaledPressure.press_abs;
        m_baroData->addPressureSample(m_scaledPressure.press_abs);
        m_baroData->temperature = m_scaledPressure.temperature;
        HFLogger::logMessage("Baro Data Avg --- p: %f   t: %d", m_baroData->pressAbs, m_baroData->temperature);
    }
}


