#ifndef CRAFTFACTORY_H
#define CRAFTFACTORY_H

#include "DeviceConnectionFactory.h"

#include "CraftConnection.h"


class CraftFactory : public DeviceConnectionFactory
{
    public:
        CraftFactory() { }
        virtual ~CraftFactory() { }

        DeviceConnection* CreateDeviceConnection() { return new CraftConnection(); }

    protected:
    private:
};

#endif // CRAFTFACTORY_H

