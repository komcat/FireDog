// PageLogin.cpp : the login gate and the page access table.
//
// This is NOT a Page. It is a gate in the frame loop (App::Run): while nobody
// is logged in it draws INSTEAD of the main window, so the menu bar, the
// F-keys, the quick strip and every floating window do not exist yet. A Page
// enum entry would have meant gating each of those surfaces separately and
// getting one of them wrong; there is nothing here to get wrong.
//
// Update() keeps running underneath, so the hardware connects and the camera
// comes up while the operator is still typing their number.
//
// Design: docs/user-access-design.md.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"   // StateColor (the refusal message)
#include "core/DutSession.h"
#include "core/Log.h"

namespace firedog {

// The access table. Operator gets the one page they work at; Technician adds
// motion, the recipe editor and the diagnostic pages they troubleshoot from;
// everything that TEACHES the machine — the frames, the detectors, the needle,
// the scripts, the transform and watch editors — is Admin, because those
// surfaces write the facts every later run is measured against.
//
// The default is Admin, deliberately. A page added later is locked until
// somebody names it here, which is the only default that cannot fail open.
UserRole MinRoleFor(Page p)
{
    switch (p) {
        // The production surface.
        case Page::Run:
            return UserRole::Operator;

        // Troubleshooting and setup of what already exists.
        case Page::Motion:
        case Page::RecipeEdit:
        case Page::Overview:
        case Page::Camera:
        case Page::Keithley:
        case Page::CLD:
        case Page::PSU:
        case Page::IO:
        case Page::Pneumatic:
        case Page::DutView:
            return UserRole::Technician;

        // Teaching the machine: WatchEdit, TransformEdit, Scripts, Align,
        // VisionTeach, AlignNominal, AlignPlace, Needle.
        default:
            return UserRole::Admin;
    }
}

// Resolve a login attempt and, on success, wire the operator into the
// traceability record. Returns false with loginErr_ set on refusal.
bool App::LogIn(const std::string& id, const std::string& pw)
{
    const ActiveUser u = Authenticate(employees_, id, pw);
    if (!u.LoggedIn()) {
        loginErr_ = "Employee number or password is wrong.";
        FD_WARN("users", "login refused for '%s'", id.c_str());
        return false;
    }

    // Where they will stand, decided BEFORE the login is committed. Run is
    // Operator-level so this resolves for every level that can sign in at all;
    // the Overview fallback only matters if MinRoleFor is ever retuned. If a
    // level can open NO page, the honest answer is to refuse rather than to
    // sign somebody in and drop them on a page the gate bounces every frame.
    Page land = currentPage_;
    if (u.role < MinRoleFor(land)) {
        if (u.role >= MinRoleFor(Page::Run))           land = Page::Run;
        else if (u.role >= MinRoleFor(Page::Overview)) land = Page::Overview;
        else {
            loginErr_ = "That level has no page it may open - see an admin.";
            FD_ERROR("users", "'%s' is %s, which can open nothing", u.id.c_str(),
                     RoleName(u.role));
            return false;
        }
    }

    user_     = u;
    loginErr_.clear();

    // The DUT record learns who is at the keyboard here and nowhere else — one
    // writer, so a record can never disagree with the title bar about whose
    // shift it was.
    DutSession::Instance().SetOperator(user_.id, user_.name, RoleKey(user_.role));

    FD_INFO("users", "%s logged in as %s", user_.Label().c_str(),
            RoleName(user_.role));

    // Without this an Operator would sign in onto whatever page the last
    // session left behind and see the refusal warning before they saw the
    // machine. Assigned directly rather than through SwitchPage: there is no
    // page to leave (the gate was up), so none of that bookkeeping applies.
    if (land != currentPage_) {
        currentPage_ = land;
        prevPage_    = land;
    }
    return true;
}

// Hand the machine back. The caller is responsible for refusing this while a
// run is in flight (see the User menu) — logging out mid-run would leave a
// program driving axes with nobody attributable to it.
void App::LogOut()
{
    FD_INFO("users", "%s logged out", user_.Label().c_str());
    user_ = ActiveUser{};
    DutSession::Instance().SetOperator("", "", "");

    // Close the privileged surfaces behind them, and clear the fields so the
    // next person does not find a number pre-typed and press Enter on it.
    showUsers_ = false;
    showJog_   = false;
    orgOpen_   = false;
    runEdit_   = false;
    // An ARMED jog must never survive a change of hands. The arm exists so that
    // moving the machine is a deliberate act by a specific person; inherited by
    // whoever signs in next it is the opposite of that. showAux_ is left alone —
    // it only displays, and its motion controls carry their own level check.
    jogArmed_ = false;
    loginId_[0] = '\0';
    loginPw_[0] = '\0';
    loginErr_.clear();
}

void App::DrawLoginScreen()
{
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(mv->Pos);
    ImGui::SetNextWindowSize(mv->Size);
    if (!ImGui::Begin("Login", nullptr,
                      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar)) {
        ImGui::End();
        return;
    }

    // A fixed-width card, centred. Two fields on a 1280x800 screen otherwise
    // spread across the whole window and read as a form to fill in rather than
    // a door to walk through.
    const float cardW = 520.0f;
    const float cardH = 300.0f;
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    ImGui::SetCursorPos(ImVec2(ImGui::GetCursorPosX() +
                                   (avail.x - cardW) * 0.5f,
                               ImGui::GetCursorPosY() +
                                   (avail.y - cardH) * 0.35f));

    ImGui::BeginChild("LoginCard", ImVec2(cardW, cardH), ImGuiChildFlags_Borders);
    ImGui::TextUnformatted("FireDog - Active Alignment");
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::TextDisabled("Sign in with your employee number.");
    ImGui::Spacing();

    const float fieldW = cardW - 220.0f;

    // Focus the number on the way in, so a badge typed straight at the screen
    // lands in the right box without a click.
    if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
    ImGui::TextUnformatted("Employee number");
    ImGui::SetNextItemWidth(fieldW);
    const bool idEnter = ImGui::InputText("##loginid", loginId_, sizeof(loginId_),
                                          ImGuiInputTextFlags_EnterReturnsTrue);

    ImGui::Spacing();
    ImGui::TextUnformatted("Password");
    // Enter on the number jumps to the password; Enter on the password submits.
    // That is what every login on the floor does, and it is what makes a badge
    // scanner that ends its scan with a newline work. The focus request has to
    // be issued BEFORE the field it targets, which is why it reads out of order
    // — the number was drawn above, so idEnter is already known here.
    if (idEnter) ImGui::SetKeyboardFocusHere();
    ImGui::SetNextItemWidth(fieldW);
    const bool pwEnter = ImGui::InputText("##loginpw", loginPw_, sizeof(loginPw_),
                                          ImGuiInputTextFlags_Password |
                                              ImGuiInputTextFlags_EnterReturnsTrue);

    ImGui::Spacing();
    ImGui::Spacing();
    const bool clicked = ImGui::Button("Sign in", ImVec2(140.0f, 0.0f));
    if (pwEnter || clicked) {
        if (LogIn(loginId_, loginPw_)) {
            loginId_[0] = '\0';
            loginPw_[0] = '\0';
        } else {
            loginPw_[0] = '\0';   // never leave a wrong password on screen
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Exit", ImVec2(100.0f, 0.0f))) host_.SetShouldClose(true);

    if (!loginErr_.empty()) {
        ImGui::Spacing();
        ImGui::TextColored(StateColor(ConnState::Failed), "%s", loginErr_.c_str());
    }

    // The one hint an empty machine needs. Shown only when there is genuinely
    // nobody to log in as — on a registered machine it would just be an
    // advertisement for the recovery account.
    if (employees_.empty()) {
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextDisabled("No employee numbers registered yet. Sign in as");
        ImGui::TextDisabled("'%s' to register them (User > Employee Numbers).",
                            kBuiltinAdminId);
    }
    ImGui::EndChild();

    ImGui::End();
}

} // namespace firedog
