#ifndef OBSERVERFACTORY_H
#define OBSERVERFACTORY_H

#include "DeviceConnectionFactory.h"

#include "ObserverConnection.h"

class ObserverFactory : public DeviceConnectionFactory
{
    public:
        ObserverFactory() { }
        virtual ~ObserverFactory() { }

        DeviceConnection* CreateDeviceConnection() { return new ObserverConnection(); }

    protected:
    private:
};

#endif // OBSERVERFACTORY_H
