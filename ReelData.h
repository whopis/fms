#ifndef REELDATA_H
#define REELDATA_H

#include <stdint.h>


class ReelData
{
    public:
        ReelData();
        virtual ~ReelData();

        int tetherPos;
        int temp;
        int volt12;
        int motorOn;
        int enc0;
        int enc1;
        
        int state;
        int faultMask;
        int errorMask;
        int tetherWarning;

        int voltTension;
        int voltMotor;
        int currMotor;
        int tempIr;
        


        int resetPosition;
        
                    
        
    protected:


    private:
};

#endif // REELDATA_H
