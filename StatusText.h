#ifndef STATUSTEXT_H
#define STATUSTEXT_H

#include <stdint.h>

class StatusText
{
    public:
        StatusText();
        virtual ~StatusText();

        uint8_t severity;
        char text[50];

    protected:
    private:
};

#endif // STATUSTEXT_H
