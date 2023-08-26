#include "HFCommand.h"

HFCommand::HFCommand(HFCommandType commandType)
{
    //ctor
    m_commandType = commandType;

    Enable = false;
    X = 0.0f;
    Y = 0.0f;
    Z = 0.0f;

    Frame = HFFRAME_CRAFT;

    AbsoluteAltitude = false;

    UseLandHeading = false;
    LandHeading = 0;

}

HFCommand::~HFCommand()
{
    //dtor
}


HFCommandType HFCommand::CommandType()
{
    return m_commandType;
}
