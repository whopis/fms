#include "GroundBaro.h"

GroundBaro::GroundBaro()
{
    //ctor
    m_baroConnection = nullptr;
}

GroundBaro::~GroundBaro()
{
    //dtor
}


void GroundBaro::AttachConnection(BaroConnection* baroConnection)
{
    m_baroConnection = baroConnection;
    m_baroConnection->AttachBaroData(&Data);
}


unsigned int GroundBaro::GetHeartbeatCount()
{
    if (m_baroConnection == nullptr)
    {
        return 0;
    }
    else
    {
        return m_baroConnection->GetHeartbeatCount();
    }
}



bool GroundBaro::DataStreamValid()
{
    if (m_baroConnection == nullptr)
    {
        return false;
    }
    else
    {
        return m_baroConnection->DataStreamValid();
    }
}

bool GroundBaro::PressureReceived()
{
    if (m_baroConnection == nullptr)
    {
        return false;
    }
    else
    {
        return m_baroConnection->PressureReceived();
    }
}



bool GroundBaro::IsConnectionActive(int maxSeconds)
{
    if (m_baroConnection == nullptr)
    {
        return false;
    }
    else
    {
        return m_baroConnection->IsConnectionActive(maxSeconds);
    }
}
