#include "BaroData.h"
#include "HFLogger.h"

BaroData::BaroData()
{
    //ctor

    pressAbs = 0.0f;
    temperature = 0;

    m_pressureFilter.setLength(60);

    m_firstSampleReceived = false;
    m_consecutiveErrorCount = 0;

}

BaroData::~BaroData()
{
    //dtor
}

int BaroData::getConsecutiveErrorCount()
{
    return m_consecutiveErrorCount;
}


void BaroData::addPressureSample(float sample)
{
    if (m_firstSampleReceived == false)
    {
        m_firstSampleReceived = true;
        HFLogger::logMessage("Received first ground baro pressure %f", sample);

        float avgPressure = m_pressureFilter.addSample(sample);
        pressAbs = avgPressure;
        HFLogger::logMessage("AvgPressure %f", avgPressure);
    }
    else
    {
        bool validSample;

        float delta = pressAbs - sample;
        if (delta < 0)
            delta *= -1;

        if ((delta / pressAbs) > 0.10)
            validSample = false;
        else
            validSample = true;


        if (validSample == true)
        {
            float avgPressure = m_pressureFilter.addSample(sample);
            pressAbs = avgPressure;
            m_consecutiveErrorCount = 0;
        }
        else
        {
            m_consecutiveErrorCount++;
            HFLogger::logMessage("Invalid Baro Sample");
            HFLogger::logMessage("PressAbs: %f  Sample: %f  delta: %f", pressAbs, sample, delta);
        }
    }
}
