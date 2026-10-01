// Employees.h : who is allowed to touch the machine, and at what level.
//
// An employee NUMBER is the traceability identity — it is what lands in the DUT
// record beside the serial, so "who built this part" has an answer that is not
// "whoever was on shift". The LEVEL is a property of that number, set once at
// registration by an admin, not something proven separately at login. That
// pairing is the whole design: the credential and the identity are the same
// fact, so a technician password cannot be lent to an unnamed pair of hands.
//
// Three deliberate choices:
//
//   * IT LIVES IN products/, NOT config/. config/ is what the repo ships — a
//     fresh build tree or a release zip starts from the committed copy — so a
//     roster there would be one release away from being replaced. products/ is
//     never shipped, so this is the only place a registration survives. It also means no xxx.local.json pair
//     is needed here: there is no shipped default to fall back to, and a second
//     file would only add a stale shadow to get confused by.
//
//   * PASSWORDS ARE PLAINTEXT, on purpose. This is weak by any security
//     standard and is not pretending otherwise: anyone who can read the file
//     can read every password. It buys a roster that can be repaired with
//     Notepad when somebody is locked out at 6am on a running line, which on
//     this machine is worth more than resistance to an attacker who, by the
//     time they have the file, already has the PC that drives the hexapods.
//
//   * ROLES ARE ORDERED, so every access check is one `>=`. A new level slots
//     in by rank instead of by editing a table of comparisons scattered across
//     the UI.
//
// The built-in admin (kBuiltinAdminId / kBuiltinAdminPassword) is checked
// BEFORE the roster and cannot be registered over, so an empty, missing or
// corrupt file is never a lockout — it is the recovery path that lets the first
// admin log in and register everybody else. It is documented, not secret.
//
// Design: docs/user-access-design.md.
#pragma once

#include <string>
#include <vector>

namespace firedog {

// Ordered by privilege: Operator < Technician < Admin. None = not logged in.
enum class UserRole { None = 0, Operator = 1, Technician = 2, Admin = 3 };

const char* RoleName(UserRole r);    // "Operator" — for display
const char* RoleKey(UserRole r);     // "operator" — the JSON spelling
UserRole    RoleFromString(const std::string& s);   // None when unrecognised

// One registered employee number.
struct Employee {
    std::string id;         // the employee number — the traceability identity
    std::string name;       // display name (optional)
    UserRole    level = UserRole::Operator;
    std::string password;   // PLAINTEXT — see the header note
    std::string added;      // NowStamp() at registration
    std::string addedBy;    // the admin who registered it
};

// Who is at the keyboard right now. role == None means the login gate is up.
struct ActiveUser {
    std::string id;
    std::string name;
    UserRole    role = UserRole::None;

    bool        LoggedIn() const { return role != UserRole::None; }
    std::string Label() const;   // "OP1023 - Somchai P." (id alone if unnamed)
};

// The recovery account. Always available, never registrable — see the note.
extern const char* const kBuiltinAdminId;         // "admin"
extern const char* const kBuiltinAdminPassword;   // "iamyourfather"

const char* EmployeesPath();   // "products/employees.json"

// Tolerant load: a missing or malformed file yields an empty roster and a
// warning, never a throw. The built-in admin can still log in either way.
std::vector<Employee> LoadEmployees();
bool SaveEmployees(const std::vector<Employee>& list);   // false if unwritable

// An employee number goes into log lines, a JSON object and a DUT record — the
// same rule as a DUT serial.
bool EmployeeIdOk(const std::string& id);

// Resolve a login attempt. The built-in admin is tried first, so the roster
// being unreadable cannot lock the machine. Returns role == None on refusal;
// the caller shows one message for every failure mode, because saying WHICH
// half was wrong tells an unregistered guesser which half to keep trying.
ActiveUser Authenticate(const std::vector<Employee>& list, const std::string& id,
                        const std::string& password);

} // namespace firedog
