#ifndef GROUNDBARO_H
#define GROUNDBARO_H


#include "BaroData.h"
#include "BaroConnection.h"

class GroundBaro
{
    public:
        GroundBaro();
        virtual ~GroundBaro();

        BaroData Data;

        void AttachConnection(BaroConnection* baroConnection);

        unsigned int GetHeartbeatCount();
        bool DataStreamValid();

        bool IsConnectionActive(int maxSeconds);
        bool PressureReceived();


    protected:

        BaroConnection* m_baroConnection;

    private:
};

#endif // GROUNDBARO_H
