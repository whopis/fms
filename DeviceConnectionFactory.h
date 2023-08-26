#ifndef DEVICECONNECTIONFACTORY_H
#define DEVICECONNECTIONFACTORY_H

#include "DeviceConnection.h"


class DeviceConnectionFactory
{
    public:
        DeviceConnectionFactory(){ }

        virtual ~DeviceConnectionFactory(){ }

        virtual DeviceConnection* CreateDeviceConnection() = 0;

    protected:
    private:
};

#endif // DEVICECONNECTIONFACTORY_H
