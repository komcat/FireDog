// FireDog.cpp : Application entry point.
//
#include "App.h"

#include "core/Paths.h"

int main()
{
    // First, before anything opens a file: the working directory becomes the
    // exe folder and the machine root is resolved (and migrated on first start).
    firedog::InitPaths();
    firedog::App app;
    return app.Run();
}
