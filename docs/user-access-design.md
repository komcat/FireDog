# User access and Operator ID traceability

*Landed 2026-08-21. Read this before changing who may open what, or before
adding a page.*

Until now FireDog had no notion of a person. Anyone who could reach the keyboard
could teach a frame, move a hexapod or edit a program, and a DUT record could
say what was built but never by whom. This is the layer that fixes both, and it
does it with one idea: **an employee number is an identity, and a level is a
property of that number.**

---

## The model

| | Where it comes from | What it answers |
|---|---|---|
| **Employee number** | picked at login, registered in `products/employees.json` | *who* built this part — it lands in the DUT record |
| **Level** | the registered property of that number | *what* they may open |
| **Password** | the registered property of that number | that this really is them |

The credential and the identity are the same fact. An earlier draft had a shared
password per level (`op`, `th`) with the number typed separately; that was
dropped because it let anyone who knew the technician password claim any
number — the record would name a person who was never there, which is worse than
no record at all.

Three levels, deliberately ordered, so every access check is one `>=` and a
fourth level would slot in by rank rather than by editing comparisons scattered
across the UI:

```cpp
enum class UserRole { None = 0, Operator = 1, Technician = 2, Admin = 3 };
```

---

## The roster: `products/employees.json`

```json
{
  "version": 1,
  "users": [
    { "id": "OP1023", "name": "Somchai P.", "level": "operator",
      "password": "…", "added": "2026-08-21T09:14:02", "added_by": "admin" }
  ]
}
```

### Why `products/`, not `config/`

**A rebuild recopies the repo's `config/` over the exe-side copy.** A roster
there would be replaced by whatever was committed, on every single build — every
registration made on the machine, gone, silently. `products/` is never copied by
`firedog_stage_runtime`, so it is the only place a registration survives.

That also means **no `xxx.local.json` pair**. That convention exists to protect a
runtime-written file from the shipped default that is about to overwrite it;
here there is no shipped default, and a second file would add nothing but a
stale shadow to get confused by.

`ListProducts()` only accepts directories containing a `manifest.json`, so a
loose file in `products/` is already invisible to it.

### Passwords are plaintext

Stated plainly because it should be: **anyone who can read that file can read
every password.** This is a deliberate trade, not an oversight. What it buys is a
roster that can be repaired with Notepad when somebody is locked out at 6am on a
running line — and by the time an attacker has the file they already have the PC
that drives the hexapods, so the hash was never protecting the machine. The one
real cost is password *reuse*: tell people not to use a password here that opens
anything else.

### The built-in admin

`admin` / `iamyourfather` is checked **before the roster is consulted at all**, so
a missing, empty or corrupt file is a nuisance rather than a lockout. It is the
recovery path that lets the first admin sign in and register everybody else, and
the reason `SaveEmployees` writes temp-then-rename is that a truncated roster
must never be able to lock out the people who are in it.

It cannot be registered over — the User Manager refuses the number `admin` —
because shadowing it would turn a forgotten password into a reinstall.

The login screen prints both the number and the password, but **only while the
roster is empty**. Naming the account without its password is the unhelpful half
of the answer: on a machine where nobody is registered there is nothing for the
recovery account to protect, and somebody who cannot get in has no way to
register the people who could. The moment anybody is registered there *is*
something to protect, and the hint disappears — otherwise it stops being a hint
and becomes an advertisement for the way around every other credential.

It is documented, not secret: the password is in this file, in the header, and
in git history. Anyone with repo access has admin on any machine. Hardening it
(require a roster admin once one exists, built-in only as fallback) is a
one-line change if the floor ever needs it.

---

## The access table

`MinRoleFor(Page)` in `ui/pages/PageLogin.cpp`:

| Level | Pages |
|---|---|
| **Operator** | Run |
| **Technician** | + Motion, Recipe Editor, Overview, Camera, Keithley, CLD, PSU, IO, Pneumatic, DUT Records |
| **Admin** | + IO Watch Editor, Transforms Editor, Scripts, Alignment, Vision teach, Nominal target, Placement, Needle |

The split is not by how dangerous a page looks but by **what it writes**.
Technician gets the surfaces that *use* what the machine already knows — jogging,
troubleshooting a meter, editing a recipe. Admin gets the surfaces that *teach*
it: the frames, the detectors, the needle calibration, the transforms. Those
write the facts every later run is measured against, and a wrong one is not a
bad part, it is a month of bad parts nobody questioned.

**The default is Admin.** A page added next year is locked until somebody names
it in the table. That is the only default that cannot fail open.

Non-page surfaces have no `Page` to look up, so their levels sit beside the table
in `App.h`: `kRoleJog` (Technician — jog commands motion), `kRoleOrg`
(Technician — the organizer edits programs), `kRoleAux` (Operator — it only
ever displays).

---

## Enforcement

### 1. The login gate is not a Page

While `user_.role == None`, `App::Run` draws `DrawLoginScreen()` **instead of**
`DrawMainWindow()`:

```cpp
if (!user_.LoggedIn()) {
    DrawLoginScreen();
} else {
    DrawMainWindow();
    …
}
```

So the menu bar, the F-keys, the quick strip and every floating window do not
exist yet. A `Page::Login` would have meant gating each of those separately and
getting one of them wrong; there is nothing here to get wrong.

`Update(dt)` still runs underneath, so the hardware connects and the camera comes
up while the operator is still typing their number — the login costs no bring-up
time.

### 2. `SwitchPage` is the choke point

Every page switch in the tree already routes through `App::SwitchPage` (an
existing enforced rule). It now refuses a page above the level and **warns**:

```cpp
if (!CanAccess(p)) { FD_WARN("users", …); return; }
```

A button or shortcut that forgets to check still cannot land anywhere it
shouldn't. It warns rather than failing silently because the log band is on
screen on the operator pages, and *"F5 did nothing"* is the bug report you get
otherwise.

### 3. Menus and buttons hide themselves

Courtesy on top of enforcement. Buttons a level cannot use are **absent, not
disabled**: an operator's quick strip is one button wide and reads as their
machine, not as six locked doors.

### 4. Windows re-check their own level

`showJog_` / `orgOpen_` / `showUsers_` are checked at the draw call as well as at
the menu item. A visibility bit outlives a page switch; a role never should.

---

## What a page gate does not catch

`SwitchPage` only guards *pages*. An audit of every other route out of the Run
page turned up four things it could never have covered, and they are the
interesting half of this work.

### The Aux window was a way around the Jog gate

`kRoleAux` is Operator because the Aux window *monitors* — live image, chart,
position labels. But it also carried **Arm jog** (the same `jogArmed_` bit the
Technician-gated Jog window arms), gantry-main Z arrows, click-to-move, and
Autofocus. Making the Jog window Technician while leaving that plate open would
have been a gate with a door beside it.

So the split runs through the window, not around it: **everything on the Aux
window that MOVES is `kRoleJog`**, the display stays Operator.

Click-to-move needed the check in *two* places — the checkbox and the click
handler — because `enableClickMove_` is persisted in `UiSettings`. A technician
who left it on would otherwise hand the next operator a live click-to-move
image with no visible control to turn it off.

### An armed jog must not survive a handover

`LogOut` clears `jogArmed_`. The arm exists so that moving the machine is a
deliberate act by a specific person; inherited by whoever signs in next it is
the exact opposite of that. `showAux_` is deliberately *not* cleared — it only
displays, and its motion controls now carry their own check.

### The one control that leaves the application

`DrawDutBlock` is shared by the Overview and Run pages, so its top-right
**Data** button (Explorer on the DUT folder) was the single control on an
operator's station that leaves FireDog entirely — and from an Explorer window,
the whole filesystem. It is Technician. The record it opens is written *for* the
operator, not *by* them.

### Helpers that mutate before they switch

`OpenVisionTeach`, `OpenVisionTeachForNominal`, `OpenAlignNominalTeach` and
`OpenAlignPlace` each set up a model, marks or selections and *then* call
`SwitchPage`. A refusal at the choke point would leave those edits behind on a
page the caller never reaches. They check at the top instead — the same shape as
the Run deck's right-click, and the reason the rule is written as *"gate a click
early only when it mutates state before switching"*.

Every caller of those four sits on an Admin page today, so this is insurance
rather than a fix. It is what makes the rule true everywhere rather than true by
coincidence.

### Deliberately left open

**`manual_adjust`'s jog window** (`WndPrompts.cpp`) is ungated. It is not free
roaming: a recipe step asked this operator to nudge the part, and the run is
blocked until they do. Gating it would mean an operator who cannot answer the
question the machine is asking them.

---

## Traceability

`DutSession::SetOperator(id, name, level)` is wired by `App::LogIn` and nowhere
else — one writer, so a record can never disagree with the menu bar about whose
shift it was.

The record carries the operator **two ways**, and both are needed:

```json
{
  "serial": "SN123",
  "operator": { "id": "OP1023", "name": "Somchai P.", "level": "operator" },
  "entries": [
    { "key": "$open",     "value": "create" },
    { "key": "$operator", "value": { "id": "OP1023", … } },
    { "key": "$run",      "value": { "kind": "program", "result": "ok",
                                     "operator": "OP1023" } }
  ]
}
```

* the **top-level object** answers *whose part is this* at a glance
* a **`$operator` entry** at open and at every handover keeps the append-only log
  honest about a record that changed hands mid-build — a log that silently
  swapped its operator would read as one continuous shift
* **`$run` carries the operator too**, because a record open across a shift
  change has runs belonging to two different people

`$operator` is appended rather than folded into `$open`'s value, so no existing
reader of that entry has to change. The key is omitted entirely when nobody is
signed in: a record with no operator key is honestly silent, where `{"id":""}`
looks like a field that was written and lost.

---

## Deliberate omissions

**No self-service password change.** Registering a number, giving it a level and
setting its password are one admin act on one surface. A level and a credential
arrive together or not at all — which is also what makes `added_by` meaningful.

**No idle auto-logout.** A timer that returns to the login screen would fire
during a long alignment, and the operator would come back to a machine asking
who they are while the hexapod is mid-scan. If it is ever wanted it belongs in
`App::Update`, gated on `!orch_.Busy()` like the sign-out below.

**Sign out is refused while `orch_.Busy()`.** Logging out mid-run would leave a
program driving axes with nobody attributable to it, and the record would name
whoever signed in next. Because it is refused, the login gate can also never
appear mid-run — which matters more than it looks: the gate replaces the whole
frame, and a run waiting on the always-on-top prompt window would be stuck
behind it exactly like the modal the UI rules already forbid.

**No modal anywhere in this layer.** Same rule. The User Manager's Remove is
gated behind an armed checkbox, like every other destructive action in this UI.
