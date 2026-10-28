// FireDog.cpp : Application entry point.
//
#include "App.h"

#include "core/Paths.h"
#include "update/AppUpdater.h"

int main()
{
    // Before anything else: during a Setup install or an update, Velopack starts
    // FireDog.exe with a hook argument and it exits right here.
    firedog::AppUpdater::RunStartupHook();
    // First, before anything opens a file: the working directory becomes the
    // exe folder and the machine root is resolved (and migrated on first start).
    firedog::InitPaths();
    firedog::AppUpdater updater;
    int rc = 0;
    {
        firedog::App app(updater);
        rc = app.Run();
    }   // ~App closes every device - BEFORE an update is handed to Update.exe
    updater.ApplyAtExit();
    return rc;
}
