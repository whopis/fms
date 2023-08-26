#include "ReelData.h"
#include "HFLogger.h"

ReelData::ReelData()
{
    //ctor
    tetherPos = 0;
    temp = 0;
    volt12 = 0;
    motorOn = 0;
    enc0 = 0;
    enc1 = 0;
    
    
    state = 0;
    faultMask = 0;
    errorMask = 0;
    tetherWarning = 0;

    voltTension = 0;
    volt12 = 0;
    voltMotor = 0;
    currMotor = 0;
    tempIr = 0;
        

}

ReelData::~ReelData()
{
    //dtor
}
