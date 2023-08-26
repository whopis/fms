#ifndef CONTROLLERFACTORY_H
#define CONTROLLERFACTORY_H

#include "DeviceConnectionFactory.h"

#include "ControllerConnection.h"

class ControllerFactory : public DeviceConnectionFactory
{
    public:
        ControllerFactory() { }
        virtual ~ControllerFactory() { }

        DeviceConnection* CreateDeviceConnection() { return new ControllerConnection(); }

    protected:
    private:
};

#endif // CONTROLLERFACTORY_H
