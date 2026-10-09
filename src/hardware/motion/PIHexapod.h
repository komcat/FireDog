// PIHexapod.h : layer 1 — direct PI GCS2 driver for a hexapod (X Y Z U V W),
// connected over TCP/IP. PIMPL keeps PI_GCS2_DLL.h / Windows out of this header.
// Multiple instances are supported (hex-left, hex-right) — each takes its own
// MotionConfig. Without HAS_PI it compiles a simulated stub with the same API.
#pragma once

#include "hardware/motion/IMotionController.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace firedog {

class PIHexapod : public IMotionController {
public:
    explicit PIHexapod(MotionConfig cfg);
    ~PIHexapod() override;

    PIHexapod(const PIHexapod&) = delete;
    PIHexapod& operator=(const PIHexapod&) = delete;

    bool Connect() override;                 // PI_ConnectTCPIP + PI_INI + start poll
    void Disconnect() override;
    bool IsConnected() const override;

    const std::string&       Name() const override;
    const std::string&       Type() const override;   // "PI"
    std::vector<std::string> Axes() const override;
    std::map<std::string, double> Positions() const override;

    bool MoveAbsolute(const std::string& axis, double position) override;   // PI_MOV
    bool MoveAbsolute(const std::vector<std::pair<std::string, double>>& targets) override;
    bool SetVelocity(double mmPerSec) override;                             // PI_VLS (system)
    bool IsSettled() const override;                                        // !PI_IsMoving
    bool IsOnTarget() const override;                                       // PI_qONT (all axes)
    bool StopAll() override;                                                // PI_STP
    void RequestFastPoll() override;

    bool     SupportsReference() const override { return true; }
    RefState Referenced() const override;                                    // cached qFRF
    bool     StartReference() override;                                      // PI_FRF (all axes)

    bool       SupportsPivot() const override { return true; }
    PivotPoint Pivot() const override;                                       // cached qSPI
    void       RequestPivotRead() override;                                  // poll thread re-reads
    bool       SetPivot(double x, double y, double z) override;              // PI_SPI R S T

    std::string LastError() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
};

} // namespace firedog
