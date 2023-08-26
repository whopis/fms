#ifndef BACKUPCONTROLLERFACTORY_H
#define BACKUPCONTROLLERFACTORY_H

#include "DeviceConnectionFactory.h"

#include "BackupControllerConnection.h"

class BackupControllerFactory : public DeviceConnectionFactory
{
    public:
        BackupControllerFactory() { }
        virtual ~BackupControllerFactory() { }

        DeviceConnection* CreateDeviceConnection() { return new BackupControllerConnection(); }

    protected:
    private:
};

#endif // BACKUPCONTROLLERFACTORY_H
