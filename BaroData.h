#ifndef BARODATA_H
#define BARODATA_H

#include <stdint.h>

#include "BoxcarFilter.h"

class BaroData
{
    public:
        BaroData();
        virtual ~BaroData();

        float pressAbs;
        int16_t temperature;

        void addPressureSample(float sample);

        int getConsecutiveErrorCount();

    protected:

        BoxcarFilter m_pressureFilter;

        bool m_firstSampleReceived;

        int m_consecutiveErrorCount;

    private:
};

#endif // GPSDATA_H
