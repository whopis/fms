#ifndef SKYBOXFACTORY_H
#define SKYBOXFACTORY_H

#include "DeviceConnectionFactory.h"

#include "SkyBoxConnection.h"

class SkyBoxFactory : public DeviceConnectionFactory
{
    public:
        SkyBoxFactory() { }
        virtual ~SkyBoxFactory() { }

        DeviceConnection* CreateDeviceConnection() { return new SkyBoxConnection(); }

    protected:
    private:
};


#endif // SKYBOXFACTORY_H
