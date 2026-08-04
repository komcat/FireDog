// RigConsole.h : the command surface, shared by the interactive REPL and the
// one-shot flags in main.cpp so both go through exactly the same code.
#pragma once

#include "RigConfigEdit.h"
#include "RigSession.h"

#include <memory>
#include <string>
#include <vector>

namespace firedog::rig {

class RigConsole {
public:
    // False if the config file could not be read at all.
    bool Init(const std::string& configPath, double connectTimeoutSec);

    // Run one command line ("connect hex-left"). False means the command failed
    // — that is what the one-shot flags turn into a non-zero exit code. `quit`
    // sets `stop` and returns true.
    bool Execute(const std::string& line, bool& stop);

    // Read typed commands from stdin until `quit` or EOF. Always returns 0.
    // This is what piped input and `--repl` use; the numbered menu in
    // RigMenu.{h,cpp} is the default face and drives Execute() the same way.
    int RunRepl();

    void PrintHelp() const;
    void PrintList() const;

    // ---- For the menu front-end -------------------------------------------
    const ConfigStore& Store() const { return store_; }
    bool               Connected() const { return session_ != nullptr; }
    // "all" or a device id; empty when nothing is connected.
    const std::string& SessionTarget() const { return sessionTarget_; }
    void               CloseSession();   // same as the `disconnect` command

private:
    // `target` is a device id, "all", or "" for "whatever is already connected".
    bool EnsureSession(const std::string& target, bool reportStatus);
    bool CmdShow(const std::vector<std::string>& tok);
    bool CmdSet(const std::vector<std::string>& tok);
    bool CmdSave(const std::vector<std::string>& tok);
    bool CmdReload();
    bool CmdReport(const std::vector<std::string>& tok);
    bool CmdConnect(const std::vector<std::string>& tok);
    bool CmdProbe(const std::vector<std::string>& tok);
    bool CmdLogLevel(const std::vector<std::string>& tok);

    ConfigStore                 store_;
    std::unique_ptr<RigSession> session_;
    std::string                 sessionTarget_;   // "all" or a device id
    double                      timeoutSec_ = 20.0;
};

} // namespace firedog::rig
