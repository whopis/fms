#ifndef BAROFACTORY_H
#define BAROFACTORY_H

#include "DeviceConnectionFactory.h"

#include "BaroConnection.h"

class BaroFactory : public DeviceConnectionFactory
{
    public:
        BaroFactory() { }
        virtual ~BaroFactory() { }

        DeviceConnection* CreateDeviceConnection() { return new BaroConnection(); }

    protected:
    private:
};


#endif // BAROFACTORY_H
