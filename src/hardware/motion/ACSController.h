// ACSController.h : layer 1 — direct ACS SPiiPlus (ACSC) driver for a gantry
// (X Y Z), connected over TCP/IP (port 701). PIMPL keeps ACSC.h / winsock out of
// this header. Without HAS_ACS it compiles a simulated stub with the same API.
#pragma once

#include "hardware/motion/IMotionController.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace firedog {

class ACSController : public IMotionController {
public:
    explicit ACSController(MotionConfig cfg);
    ~ACSController() override;

    ACSController(const ACSController&) = delete;
    ACSController& operator=(const ACSController&) = delete;

    bool Connect() override;                 // acsc_OpenCommEthernet + enable axes
    void Disconnect() override;
    bool IsConnected() const override;

    const std::string&       Name() const override;
    const std::string&       Type() const override;   // "ACS"
    std::vector<std::string> Axes() const override;
    std::map<std::string, double> Positions() const override;

    bool MoveAbsolute(const std::string& axis, double position) override;   // acsc_ToPointM+GoM
    bool MoveAbsolute(const std::vector<std::pair<std::string, double>>& targets) override;
    bool SetVelocity(double mmPerSec) override;                             // acsc_SetVelocity (all axes)
    bool StopAll() override;                                                // acsc_KillAll
    bool SupportsAxisEnable() const override { return true; }
    std::map<std::string, bool> AxisEnabled() const override;               // ACSC_MST_ENABLE
    bool SetAxisEnabled(const std::string& axis, bool on) override;         // acsc_Enable/Disable
    bool SupportsBuffers() const override { return true; }
    bool RunBuffer(int buffer, const std::string& label = {}) override;     // acsc_RunBuffer
    bool StopBuffer(int buffer) override;                                   // acsc_StopBuffer
    bool BufferRunning(int buffer) const override;                          // ACSC_PST_RUN, polled
    void RequestFastPoll() override;

    std::string LastError() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
};

} // namespace firedog
