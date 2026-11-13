// PorExport.h : "Export master" - copy a master machine's product and the
// machine files the compare reads into an EMPTY folder, laid out like a machine
// root, so it can travel on a stick or a share and be compared anywhere
// (docs/por-design.md §5.1).
//
// Reads the master, writes only under `dest`:
//     <dest>/products/<product>/...           (minus *.bak and the *.history.jsonl journal)
//     <dest>/config/hardware.json, maps.local.json, motion_transforms.json
//     <dest>/por_export.json                  (what, from where, when, by which build)
#pragma once

#include <string>
#include <vector>

namespace firedog {

// False with *err when nothing was written (a refusal) or the copy failed part
// way; *log lists every file written either way.
bool ExportMaster(const std::string& masterRoot, const std::string& product,
                  const std::string& dest, std::vector<std::string>* log, std::string* err);

} // namespace firedog
