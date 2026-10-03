// Employees.cpp
#include "core/Employees.h"

#include "core/Log.h"
#include "core/Paths.h"

#include <exception>
#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

namespace firedog {

const char* const kBuiltinAdminId       = "admin";
const char* const kBuiltinAdminPassword = "iamyourfather";

std::string EmployeesPath() { return MachinePath("products/employees.json"); }

const char* RoleName(UserRole r)
{
    switch (r) {
        case UserRole::Operator:   return "Operator";
        case UserRole::Technician: return "Technician";
        case UserRole::Admin:      return "Admin";
        default:                   return "(none)";
    }
}

const char* RoleKey(UserRole r)
{
    switch (r) {
        case UserRole::Operator:   return "operator";
        case UserRole::Technician: return "technician";
        case UserRole::Admin:      return "admin";
        default:                   return "none";
    }
}

UserRole RoleFromString(const std::string& s)
{
    if (s == "operator")   return UserRole::Operator;
    if (s == "technician") return UserRole::Technician;
    if (s == "admin")      return UserRole::Admin;
    return UserRole::None;
}

std::string ActiveUser::Label() const
{
    if (id.empty()) return "(nobody)";
    return name.empty() ? id : id + " - " + name;
}

bool EmployeeIdOk(const std::string& id)
{
    if (id.empty() || id.size() > 64) return false;
    for (const char c : id) {
        const bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
                        (c >= 'A' && c <= 'Z') || c == '-' || c == '_' || c == '.';
        if (!ok) return false;
    }
    return true;
}

std::vector<Employee> LoadEmployees()
{
    std::vector<Employee> out;

    const std::string pathStr = EmployeesPath();
    const char* const kPath   = pathStr.c_str();
    std::ifstream f(pathStr);
    if (!f.is_open()) {
        // Not an error: a machine that has never registered anybody is exactly
        // the state the built-in admin exists for.
        FD_INFO("users", "no roster at %s yet - built-in admin only", kPath);
        return out;
    }
    try {
        nlohmann::json j;
        f >> j;
        const nlohmann::json& arr = j.contains("users") ? j["users"] : j;
        if (!arr.is_array()) {
            FD_WARN("users", "%s has no 'users' array - roster ignored", kPath);
            return out;
        }
        for (const nlohmann::json& e : arr) {
            if (!e.is_object()) continue;
            Employee emp;
            emp.id       = e.value("id", std::string());
            emp.name     = e.value("name", std::string());
            emp.level    = RoleFromString(e.value("level", std::string("operator")));
            emp.password = e.value("password", std::string());
            emp.added    = e.value("added", std::string());
            emp.addedBy  = e.value("added_by", std::string());
            // A row that cannot log in is worse than no row: it occupies its
            // number and silently refuses. Drop it loudly instead.
            if (!EmployeeIdOk(emp.id)) {
                FD_WARN("users", "roster row with invalid id '%s' skipped",
                        emp.id.c_str());
                continue;
            }
            if (emp.level == UserRole::None) {
                FD_WARN("users", "'%s' has an unrecognised level - skipped",
                        emp.id.c_str());
                continue;
            }
            out.push_back(std::move(emp));
        }
    } catch (const std::exception& ex) {
        FD_WARN("users", "%s is unreadable (%s) - built-in admin only", kPath,
                ex.what());
        return {};
    }
    FD_INFO("users", "%zu employee number(s) registered", out.size());
    return out;
}

bool SaveEmployees(const std::vector<Employee>& list)
{
    nlohmann::json arr = nlohmann::json::array();
    for (const Employee& e : list) {
        nlohmann::json o;
        o["id"]       = e.id;
        o["name"]     = e.name;
        o["level"]    = RoleKey(e.level);
        o["password"] = e.password;
        o["added"]    = e.added;
        o["added_by"] = e.addedBy;
        arr.push_back(std::move(o));
    }
    nlohmann::json j;
    j["version"] = 1;
    j["users"]   = std::move(arr);

    // products/ is created by the first product; a machine may not have one yet.
    const std::string pathStr = EmployeesPath();
    const char* const kPath   = pathStr.c_str();
    std::error_code ec;
    std::filesystem::create_directories(
        std::filesystem::path(pathStr).parent_path(), ec);

    // Same write-temp-then-rename as the DUT record: a crash mid-write must not
    // leave a truncated roster, which would lock out everyone but the built-in
    // admin.
    const std::string tmp = std::string(kPath) + ".tmp";
    {
        std::ofstream f(tmp);
        if (!f.is_open()) {
            FD_ERROR("users", "cannot write %s", tmp.c_str());
            return false;
        }
        f << j.dump(2) << '\n';
        if (!f.good()) {
            FD_ERROR("users", "write failed for %s", tmp.c_str());
            return false;
        }
    }
    std::filesystem::rename(tmp, kPath, ec);
    if (ec) {
        FD_ERROR("users", "rename to %s failed: %s", kPath, ec.message().c_str());
        return false;
    }
    return true;
}

ActiveUser Authenticate(const std::vector<Employee>& list, const std::string& id,
                        const std::string& password)
{
    ActiveUser u;

    // FIRST, before the roster is consulted at all: this is what makes a
    // missing or corrupt file a nuisance rather than a lockout.
    if (id == kBuiltinAdminId) {
        if (password == kBuiltinAdminPassword) {
            u.id   = kBuiltinAdminId;
            u.name = "Built-in administrator";
            u.role = UserRole::Admin;
        }
        return u;   // the built-in number is never a roster number
    }

    for (const Employee& e : list) {
        if (e.id != id) continue;
        if (e.password != password) break;   // right number, wrong password
        u.id   = e.id;
        u.name = e.name;
        u.role = e.level;
        return u;
    }
    return u;
}

} // namespace firedog
