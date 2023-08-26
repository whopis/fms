#ifndef DISPLAY_H
#define DISPLAY_H

#include <ncurses.h>

#include <time.h>

#include "SystemState.h"
#include "Craft.h"
#include "GroundGPS.h"
#include "Controller.h"
#include "GroundBaro.h"
#include "CommunicationsHub.h"


class Display
{
    public:
        Display();
        virtual ~Display();


        void Initialize(systemState_t* systemState);

        void AttachCraft(Craft* craft, deviceConnectionState* craftConnectionState);
        void AttachController(Controller* controller, deviceConnectionState* controllerConnectionState);
        void AttachGroundGps(GroundGPS* groundGps, deviceConnectionState* groundGpsConnectionState);
        void AttachGroundBaro(GroundBaro* groundBaro, deviceConnectionState* groundBaroConnectionState);
        void AttachCommunicationsHub(CommunicationsHub* communicationsHub);


        void Update();

    protected:
    private:


        WINDOW* m_titleWindow;
        WINDOW* m_systemWindow;
        WINDOW* m_craftWindow;
        WINDOW* m_controllerWindow;
        WINDOW* m_groundGPSWindow;
        WINDOW* m_groundBaroWindow;
        WINDOW* m_communicationsHubWindow;

        Craft* m_craft;
        GroundGPS* m_groundGps;
        Controller* m_controller;
        GroundBaro* m_groundBaro;

        CommunicationsHub* m_communicationsHub;


        time_t m_systemStartTime;


        void UpdateSystemStatus();
        void UpdateCraft();
        void UpdateGroundGps();
        void UpdateController();
        void UpdateCommunicationsHub();
        void UpdateGroundBaro();


        systemState_t* m_systemState;
        deviceConnectionState_t* m_craftConnectionState;
        deviceConnectionState_t* m_groundGpsConnectionState;
        deviceConnectionState_t* m_controllerConnectionState;
        deviceConnectionState_t* m_groundBaroConnectionState;

        bool m_initialized;

        void DisplayStatus(WINDOW* win, generalState_t state);
        void wprintSystemState(WINDOW* win);

        int counter;
};

#endif // DISPLAY_H
