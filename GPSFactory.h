#ifndef GPSFACTORY_H
#define GPSFACTORY_H

#include "DeviceConnectionFactory.h"

#include "GPSConnection.h"

class GPSFactory : public DeviceConnectionFactory
{
    public:
        GPSFactory() { }
        virtual ~GPSFactory() { }

        DeviceConnection* CreateDeviceConnection() { return new GPSConnection(); }

    protected:
    private:
};


#endif // GPSFACTORY_H
