// WndEmployees.cpp : the User Manager — register an employee number, give it a
// level, set its password. Admin only.
//
// This is the ONE place a password is written. You said registration must be an
// admin's act, so there is no self-service change form anywhere: a level and a
// credential arrive together, from one surface, or not at all. That is also
// what makes `added_by` meaningful — every row can name the admin who granted
// it.
//
// No modal, by house rule: a modal blocks input to every other window,
// including the always-on-top prompt an engine worker may be blocked on. Remove
// is gated behind an armed checkbox instead, like every other destructive
// action in this UI.
//
// Design: docs/user-access-design.md.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"   // StateColor (the refusal message)
#include "core/Log.h"
#include "core/Product.h"   // NowStamp()

#include <cstddef>
#include <cstring>

namespace firedog {

namespace {

// The level combo, in privilege order so the list reads as a ladder.
constexpr const char* kLevelNames[3] = {"Operator", "Technician", "Admin"};

UserRole LevelFromIndex(int i)
{
    switch (i) {
        case 1:  return UserRole::Technician;
        case 2:  return UserRole::Admin;
        default: return UserRole::Operator;
    }
}

int IndexFromLevel(UserRole r)
{
    switch (r) {
        case UserRole::Technician: return 1;
        case UserRole::Admin:      return 2;
        default:                   return 0;
    }
}

void CopyInto(char* dst, std::size_t cap, const std::string& src)
{
    const std::size_t n = src.size() < cap - 1 ? src.size() : cap - 1;
    std::memcpy(dst, src.data(), n);
    dst[n] = '\0';
}

} // namespace

void App::DrawEmployeeManager()
{
    // Belt and braces: the menu item is already Admin-gated, but this window's
    // visibility bit outlives a page switch and a role never should.
    if (!CanUse(UserRole::Admin)) { showUsers_ = false; return; }

    ImGui::SetNextWindowSize(ImVec2(880.0f, 480.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Employee Numbers", &showUsers_)) {
        ImGui::End();
        return;
    }

    // Keep the selection honest: a row removed out from under it would other-
    // wise point at whoever moved up into the slot, and the next Apply would
    // reset THEIR password.
    if (usrSel_ >= static_cast<int>(employees_.size())) usrSel_ = -1;

    const float formW = 340.0f;

    // ---- Left: the roster ----
    ImGui::BeginChild("UsrList",
                      ImVec2(ImGui::GetContentRegionAvail().x - formW - 8.0f, 0.0f),
                      ImGuiChildFlags_Borders);
    if (employees_.empty()) {
        ImGui::TextDisabled("No employee numbers registered.");
        ImGui::TextDisabled("Fill in the form and press Register.");
    } else if (ImGui::BeginTable("##usrtbl", 5,
                                 ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders |
                                     ImGuiTableFlags_ScrollY)) {
        ImGui::TableSetupColumn("Number");
        ImGui::TableSetupColumn("Name");
        ImGui::TableSetupColumn("Level");
        ImGui::TableSetupColumn("Registered");
        ImGui::TableSetupColumn("By");
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableHeadersRow();

        for (int i = 0; i < static_cast<int>(employees_.size()); ++i) {
            const Employee& e = employees_[static_cast<std::size_t>(i)];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::PushID(i);
            if (ImGui::Selectable(e.id.c_str(), usrSel_ == i,
                                  ImGuiSelectableFlags_SpanAllColumns)) {
                usrSel_       = i;
                usrArmRemove_ = false;   // a new selection is never pre-armed
                usrErr_.clear();
                usrNote_.clear();
                CopyInto(usrId_, sizeof(usrId_), e.id);
                CopyInto(usrName_, sizeof(usrName_), e.name);
                usrPw_[0]  = '\0';       // blank = keep the current password
                usrLevel_  = IndexFromLevel(e.level);
            }
            ImGui::PopID();
            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(e.name.empty() ? "-" : e.name.c_str());
            ImGui::TableSetColumnIndex(2);
            ImGui::TextUnformatted(RoleName(e.level));
            ImGui::TableSetColumnIndex(3);
            ImGui::TextUnformatted(e.added.empty() ? "-" : e.added.c_str());
            ImGui::TableSetColumnIndex(4);
            ImGui::TextUnformatted(e.addedBy.empty() ? "-" : e.addedBy.c_str());
        }
        ImGui::EndTable();
    }
    ImGui::EndChild();

    ImGui::SameLine();

    // ---- Right: register / edit ----
    ImGui::BeginChild("UsrForm", ImVec2(formW, 0.0f), ImGuiChildFlags_Borders);

    const bool editing = usrSel_ >= 0;
    ImGui::TextUnformatted(editing ? "Edit employee number" : "Register employee number");
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::TextUnformatted("Employee number");
    ImGui::SetNextItemWidth(-1.0f);
    // The number is the identity a DUT record was written against — renaming it
    // would orphan every record that already names it. Change of number = a new
    // registration and a removal, deliberately two acts.
    ImGui::BeginDisabled(editing);
    ImGui::InputText("##usrid", usrId_, sizeof(usrId_));
    ImGui::EndDisabled();

    ImGui::Spacing();
    ImGui::TextUnformatted("Name (optional)");
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputText("##usrname", usrName_, sizeof(usrName_));

    ImGui::Spacing();
    ImGui::TextUnformatted("Level");
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::Combo("##usrlevel", &usrLevel_, kLevelNames, 3);

    ImGui::Spacing();
    ImGui::TextUnformatted(editing ? "Password (blank = unchanged)" : "Password");
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputText("##usrpw", usrPw_, sizeof(usrPw_), ImGuiInputTextFlags_Password);

    ImGui::Spacing();
    ImGui::Spacing();

    if (ImGui::Button(editing ? "Apply" : "Register", ImVec2(120.0f, 0.0f))) {
        usrErr_.clear();
        usrNote_.clear();
        const std::string id  = usrId_;
        const std::string pw  = usrPw_;
        const UserRole    lvl = LevelFromIndex(usrLevel_);

        if (editing) {
            Employee& e = employees_[static_cast<std::size_t>(usrSel_)];
            // A password is only replaced when one was typed; blank means keep,
            // which is what lets a level be changed without knowing the old one.
            if (!pw.empty()) e.password = pw;
            e.name  = usrName_;
            e.level = lvl;
            if (SaveEmployees(employees_)) {
                usrNote_ = "Saved.";
                FD_INFO("users", "%s updated '%s' (%s%s)", user_.id.c_str(),
                        e.id.c_str(), RoleName(e.level),
                        pw.empty() ? "" : ", new password");
            } else {
                usrErr_ = "Could not write the roster file.";
            }
            usrPw_[0] = '\0';
        } else if (!EmployeeIdOk(id)) {
            usrErr_ = "Use letters, digits, - _ . (max 64).";
        } else if (id == kBuiltinAdminId) {
            // Registering over it would shadow the recovery account and turn a
            // forgotten password into a reinstall.
            usrErr_ = "'admin' is the built-in recovery account.";
        } else if (pw.empty()) {
            usrErr_ = "A password is required.";
        } else {
            bool dup = false;
            for (const Employee& e : employees_)
                if (e.id == id) { dup = true; break; }
            if (dup) {
                usrErr_ = "That number is already registered.";
            } else {
                Employee e;
                e.id       = id;
                e.name     = usrName_;
                e.level    = lvl;
                e.password = pw;
                e.added    = NowStamp();
                e.addedBy  = user_.id;
                employees_.push_back(std::move(e));
                if (SaveEmployees(employees_)) {
                    usrNote_ = "Registered.";
                    FD_INFO("users", "%s registered '%s' as %s", user_.id.c_str(),
                            id.c_str(), RoleName(lvl));
                    usrId_[0]   = '\0';
                    usrName_[0] = '\0';
                    usrPw_[0]   = '\0';
                    usrLevel_   = 0;
                } else {
                    employees_.pop_back();   // never claim a write that failed
                    usrErr_ = "Could not write the roster file.";
                }
            }
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("New", ImVec2(90.0f, 0.0f))) {
        usrSel_       = -1;
        usrArmRemove_ = false;
        usrId_[0]     = '\0';
        usrName_[0]   = '\0';
        usrPw_[0]     = '\0';
        usrLevel_     = 0;
        usrErr_.clear();
        usrNote_.clear();
    }

    if (editing) {
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        ImGui::Checkbox("Confirm", &usrArmRemove_);
        ImGui::SameLine();
        ImGui::BeginDisabled(!usrArmRemove_);
        if (ImGui::Button("Remove", ImVec2(120.0f, 0.0f))) {
            const std::ptrdiff_t at = static_cast<std::ptrdiff_t>(usrSel_);
            const Employee    back = employees_[static_cast<std::size_t>(usrSel_)];
            const std::string gone = back.id;
            employees_.erase(employees_.begin() + at);
            if (SaveEmployees(employees_)) {
                FD_INFO("users", "%s removed '%s'", user_.id.c_str(), gone.c_str());
                usrNote_      = "Removed " + gone + ".";
                usrSel_       = -1;
                usrArmRemove_ = false;
                usrId_[0]     = '\0';
                usrName_[0]   = '\0';
                usrPw_[0]     = '\0';
            } else {
                employees_.insert(employees_.begin() + at, back);
                usrErr_ = "Could not write the roster file.";
            }
        }
        ImGui::EndDisabled();
        // Removing yourself is allowed and survives: the session in progress is
        // already authenticated, and the built-in admin means the machine can
        // always be got back into. It costs the next login, which is the point.
        if (usrSel_ >= 0 &&
            employees_[static_cast<std::size_t>(usrSel_)].id == user_.id)
            ImGui::TextDisabled("This is the number you are signed in with.");
    }

    if (!usrErr_.empty()) {
        ImGui::Spacing();
        ImGui::TextColored(StateColor(ConnState::Failed), "%s", usrErr_.c_str());
    } else if (!usrNote_.empty()) {
        ImGui::Spacing();
        ImGui::TextDisabled("%s", usrNote_.c_str());
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::TextDisabled("Stored in %s", EmployeesPath().c_str());
    ImGui::TextDisabled("Passwords are stored as plain text.");

    ImGui::EndChild();
    ImGui::End();
}

} // namespace firedog
