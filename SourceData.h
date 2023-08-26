#ifndef SOURCEDATA_H
#define SOURCEDATA_H

#include <stdint.h>


enum SourceDeviceType {
    Source_FMS = 0,
    Source_Controller = 1,
    Source_GroundGps = 2,
    Source_Craft = 3,
    Source_Target = 4,
    Source_TK = 5
};

class SourceData
{
    public:
        SourceData();
        virtual ~SourceData();

        static uint8_t SystemId(SourceDeviceType device);
        static uint8_t ComponentId(SourceDeviceType device);

    protected:
    private:
};

#endif // SOURCEDATA_H
