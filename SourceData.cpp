#include "SourceData.h"

SourceData::SourceData()
{
    //ctor
}

SourceData::~SourceData()
{
    //dtor
}



uint8_t SourceData::SystemId(SourceDeviceType device)
{
    switch (device)
    {
        case Source_FMS:
            return 253;
            break;
        case Source_Controller:
            return 253;
            break;
        case Source_GroundGps:
            return 253;
            break;
        case Source_Craft:
            return 253;
            break;
        case Source_Target:
            return 253;
            break;
        case Source_TK:
            return 253;
            break;
        default:
            return 253;
            break;
    }
}

uint8_t SourceData::ComponentId(SourceDeviceType device)
{
    switch (device)
    {
        case Source_FMS:
            return 200;
            break;
        case Source_Controller:
            return 150;
            break;
        case Source_GroundGps:
            return 110;
            break;
        case Source_Craft:
            return 100;
            break;
        case Source_Target:
            return 101;
            break;
        case Source_TK:
            return 102;
            break;
        default:
            return 190;
            break;
    }
}
