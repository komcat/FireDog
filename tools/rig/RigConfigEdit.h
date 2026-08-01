// RigConfigEdit.h : the rig's working copy of config/hardware.json.
//
// One flat, addressable view over the config's per-kind vectors: every device
// is named by its config id (motion/io/camera) or name (instruments), which is
// the same string the operator types at the prompt. `Set` edits memory and
// marks the entry dirty; `Save` writes just that entry back through the
// matching HardwareConfig::Save* read-modify-write helper, so no key the rig
// does not own (pin maps, io_watch, pneumatic_slides) is ever rewritten.
//
// Nothing here touches hardware — a live session keeps the config it was
// started with, so an edit only takes effect on the next connect.
#pragma once

#include "hardware/HardwareConfig.h"

#include <cstddef>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace firedog::rig {

enum class DeviceKind { Motion, Io, Camera, Keithley, Cld, Psu };

const char* KindName(DeviceKind k);

// A device's address inside a ConfigStore. Indices are rebuilt by Load(), so
// hold one only for the duration of a command.
struct DeviceRef {
    DeviceKind  kind  = DeviceKind::Motion;
    std::size_t index = 0;
    std::string id;      // how the operator refers to it
};

class ConfigStore {
public:
    // Replaces the whole working copy and clears every dirty mark. False if the
    // file could not be read (HardwareConfig::Load is tolerant, so an empty
    // result means "nothing configured", not an error).
    bool Load(const std::string& path);

    const std::string&    Path() const { return path_; }
    const HardwareConfig& Config() const { return cfg_; }

    const std::vector<DeviceRef>& Devices() const { return devices_; }
    // Exact match on id/name first, then case-insensitive.
    std::optional<DeviceRef> Find(const std::string& id) const;

    // "PI motion  hex-left  192.168.0.10:50000  enabled" — one line for `list`.
    std::string Summary(const DeviceRef& d) const;
    // Editable fields as (name, current value) for `show`.
    std::vector<std::pair<std::string, std::string>> Fields(const DeviceRef& d) const;

    // Parse `value` for `field` and assign it. On false, `err` says which of the
    // two was wrong. On true the entry is marked dirty.
    bool Set(const DeviceRef& d, const std::string& field, const std::string& value,
             std::string& err);

    bool Dirty(const DeviceRef& d) const;
    bool AnyDirty() const { return !dirty_.empty(); }
    std::vector<DeviceRef> DirtyDevices() const;

    // Write one entry back to Path(). Clears its dirty mark on success.
    bool Save(const DeviceRef& d, std::string& err);

private:
    std::string KeyOf(const DeviceRef& d) const;
    void        RebuildIndex();

    std::string           path_;
    HardwareConfig        cfg_;
    std::vector<DeviceRef> devices_;
    std::set<std::string>  dirty_;
};

} // namespace firedog::rig
