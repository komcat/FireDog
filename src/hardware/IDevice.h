// IDevice.h : layer 1 — common interface every instrument adapter implements.
#pragma once

#include <string>

namespace firedog {

enum class ConnState { Disconnected, Connecting, Connected, Failed };

class IDevice {
public:
    virtual ~IDevice() = default;

    virtual const std::string& Name() const = 0;
    virtual bool  Connect()    = 0;   // blocking; returns true on success
    virtual void  Disconnect() = 0;
    virtual ConnState State() const = 0;
};

} // namespace firedog
