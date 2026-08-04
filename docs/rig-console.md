# FireDogRig — the console bring-up rig

*A second executable next to `FireDog.exe`: connect the machine's hardware and
read it back from a terminal, edit the connection fields of
`config/hardware.json` without launching the GUI, and write the whole
configuration out as a readable text file to hand to another team.*

## Why it exists

Finding out whether a machine is reachable used to mean starting the full
GLFW+ImGui client and walking its pages — heavy over a remote session, and a
wrong `ip_address` cost an app restart per attempt. The rig answers the same
question from a prompt.

What makes its verdict trustworthy is that it does **not** re-implement anything:
it constructs `HardwareManager`, hands it to `Workflow`, and calls
`Workflow::Start()` — the identical bring-up sequence `App::Run` performs. If the
rig connects, the client will connect.

## Read-only by construction

The rig calls getters and cached snapshots only. There is no command to move an
axis, write an IO output, extend a slide, switch a PSU channel or the laser, or
change a setpoint. Pointing it at a live machine cannot move anything.

(Actuation was deliberately left out: a bring-up tool that can crash a stage is
the wrong trade. Commanding hardware stays in the client, behind its armed
checkboxes.)

## Build & run

Same presets, same output directory as the client:

```bash
cmake --preset x64-debug
out/build/x64-debug/FireDogRig.exe
```

Sources live in `tools/rig/` — deliberately outside `src/`, because the FireDog
target globs `src/**` and would otherwise swallow the rig's `main()`.

The rig switches its working directory to its own folder at startup, so it finds
`config/hardware.json` and writes `logs/` correctly however it was invoked. The
usual `FD_*` log lines go to stdout (colored) and to the same daily log file the
client writes.

## The menu

Launched with no arguments on a real terminal, the rig shows a numbered menu:
pick a number, press Enter.

```
  ==========================================================================
   FireDogRig - hardware bring-up            connected: every enabled device
   config/hardware.json   (10 devices, 1 unsaved)
  ==========================================================================
    1) Connect all              5) List devices
    2) Connect one device...    6) Show / edit a device...
    3) Probe (read-back)...     7) Save changes to config
    4) Disconnect               8) Reload config (drop edits)
  --------------------------------------------------------------------------
    9) Connection status       10) Log level...
   11) Write config report (txt, for handover)
    c) Type a command           h) Command help
    q) Quit
  select>
```

`2`, `3` and `6` open a numbered device picker (typing the name works too); `6`
then lists the device's fields numbered, and picking one prompts for the new
value showing the current one, with blank to cancel. Enter on its own reprints
the menu.

The menu is **reprinted, never cleared** — deliberately. The rig's other half is
the driver log streaming past (`[pi] hex-left connected @ 192.168.0.10:50000`),
and a full-screen TUI would have to fight it for the terminal. This also means
the whole session stays in scrollback. That is why there is no FTXUI/ncurses
dependency here: "press 1, 2, 3, Enter" needs nothing beyond `std::getline`.

Every menu choice builds the same command string the typed prompt takes, so the
menu, `--repl` and the one-shot flags all run one code path.

## Commands

`c` from the menu, or the whole session with `--repl` (which is also the
automatic choice when stdin is a pipe or a file, so `echo probe all | FireDogRig`
still works):

```
list                        devices in the config (* = unsaved edit)
show <device>               editable fields and their current values
set <device> <field> <val>  edit in memory (takes effect on the next connect)
save [<device>|all]         write edited entries back to the config file
reload                      re-read the config file, dropping unsaved edits
report [<file>]             write the whole configuration as readable text

connect [<device>|all]      bring up hardware the way the client does
status                      connection state of everything brought up
probe [<device>|all]        connect (if needed) and print live read-back
disconnect                  tear the session down (closes the device sessions)

loglevel <trace|debug|info|warn|error>
help / quit
```

A device is named by its config `id` (`hex-left`, `io-bottom`, `main_camera`) or
its `name` (`IOBottom`, `Keithley_6482`, `PSU3`) — whichever you have in front of
you.

### connect vs connect \<device\>

`connect all` passes the loaded config through untouched: motion, IO, pneumatic
slides, io_watch, safety_watch, camera, Keithley, CLD, PSU — the client's exact
sequence, including `HardwareManager::ConnectAllAsync` for the simulated fleet.

`connect <device>` starts a copy of the config with every *other* device disabled,
**and with the pneumatic slides, io_watch and safety_watch disabled too**: those
reference pins by name across the whole IO fleet, and in a one-device fleet they
could only log resolution failures.

A session is torn down and rebuilt on every connect — that teardown is what
closes the VISA and TCP sessions.

### probe

`probe <device>` re-connects to that device first; a bare `probe` reports on
whatever session is already up (connecting everything if none is). It requests a
fast poll from the motion drivers, waits ~0.8 s for every poll thread to produce
a sample, then prints per kind:

| Device | Read back |
|---|---|
| motion | type, axes, live per-axis positions, settled / on-target, last error |
| io | input and output bitmaps as one line per pin with its config name, latch |
| slides | position state, output + both sensor bits, whether the IO cache is fresh |
| camera | model, serial, IP, frame size, measured fps, exposure |
| keithley | IDN, VISA resource, NPLC, range, bias + output state, live ch1/ch2 |
| cld101x | IDN, TEC/laser on-off, temperature (+validity), setpoints, live streams |
| psu | IDN, per channel output state, setpoints, measured V/I, live streams |
| vision | HALCON: available / compiled-in / runtime / licensed, version, root, licence dir + files, error |

The `vision` block is machine state rather than a configured device, so it prints
unconditionally — "does this PC have a valid HALCON licence?" is a bring-up
question, and it needs no hardware to answer. See `docs/vision-layer.md`.

Live values come from the DataBus topics in `core/Topics.h`; `(no samples)` means
the poll thread has not produced one yet, which is itself a useful answer.

Slide state machines only advance while `Workflow::Update` is ticking, which the
rig does during a connect or probe wait — there is no background ticker thread.

## Editing the config

`set` mutates the in-memory config and marks the entry dirty; `save` writes just
that entry back through the matching `HardwareConfig::Save*` read-modify-write
helper, so every key the rig does not own is preserved — notably
`io_devices[].pins`.

Editable fields:

| Kind | Fields |
|---|---|
| motion | `name` `controller_type` `axes` `ip_address` `port` `device_id` `enabled` |
| io | `name` `type` `ip_address` `port` `device_id` `input_count` `output_count` `poll_interval_ms` `enabled` `simulated` |
| camera | `camera_type` `connection_type` `ip_address` `serial_number` `display_name` `device_index` `exposure_us` `pixel_to_mm_x` `pixel_to_mm_y` `enabled` `auto_connect` |
| keithley | `visa_resource` `channel_count` `measurement_nplc` `auto_range` `fixed_range` `poll_interval_ms` `bias_voltage` `enabled` |
| cld | `visa_resource` `default_temperature` `default_laser_current` `poll_interval_ms` `enabled` |
| psu | `visa_resource` `channels` `poll_interval_ms` `enabled` |

`controller_type`, the IO `type` and `connection_type` are validated against
their allowed values: a typo there does not fail loudly at connect (Workflow logs
a warning and skips the device), so it is rejected at the edit instead.

Not editable here — they stay in the file or the GUI: pin name maps,
`pneumatic_slides`, `io_watch`, `safety_watch`, and the Keithley's
`enabled_channels`. The **report** below prints all of them anyway: what you
cannot edit here, you can still hand over.

Edits never apply to a live session: the session keeps the config it was started
with, so changing an IP needs `disconnect` then `connect`.

**The rebuild caveat** (the same one the IO Watch editor prints): the rig writes
`config/hardware.json` *next to the exe*, and a rebuild recopies the repo's
`config/` over it. Mirror durable edits into the repo file — or run the rig with
`--config` pointed at the repo copy.

## The handover report

`report [<file>]` (menu item 11, `--report`) writes **one plain-text file
describing the entire configuration** — for a team that has to set the machine
up and has neither the GUI nor a JSON editor in front of them. Default name
`firedog-config-report.txt`, next to the exe.

It needs no hardware: it describes the loaded config, never a live device.

Ten sections: a summary, a **connection map** (every IP, port and VISA resource
in one table — the list you check against the switch), motion controllers with
each one's axis transform written out as `X -> +Z   Y -> +Y   Z -> +X` rather
than as a matrix, IO devices with **every pin number and its name** (including
pins with no name, and a warning on a name past the declared pin count),
pneumatic slides, `io_watch` bindings, `safety_watch` signals with their alert
text, cameras with their mm-per-pixel calibration, instruments, and a closing
page of notes on where the file lives, how to change it, which subnets the PC
needs an adapter on, **where to download NI-VISA** and how to verify the
instruments in NI-MAX, and the bring-up order.

Two things the report is careful about:

- It reports **unsaved rig edits** as edited, marks them `*`, and says so at the
  top — a handover file must never quietly disagree with the machine.
- The instrument notes are **derived from the resource strings**, not boilerplate.
  Every transport goes through NI-VISA (`Keithley6482.cpp` opens the instrument
  with `viOpen`, whatever the cable), so the download link is always there; but
  NI-488.2 and a GPIB adapter are only called for when a `GPIB0::…` resource
  exists, and a `TCPIP0::…` one instead gets the static-IP and NI-MAX *Add
  Network Device* steps — and its address joins the subnet list in note 3, which
  is the easiest one to miss because it hides inside a VISA string.
- It reads `motion_transforms.json` **beside the hardware file**, so
  `--config ../../config/hardware.json --report` describes the repo copy, not
  the exe-side one. Entries there for devices absent from this config are
  listed as ignored.

`--report` is read-only, like every other rig command: it writes the report file
and nothing else.

## One-shot flags

No REPL; exits 0 only if every targeted device reached `Connected`, so it works
in a script or a bring-up checklist.

```bash
FireDogRig --list
FireDogRig --report                      # firedog-config-report.txt next to the exe
FireDogRig --report D:\handover\machine-A.txt
FireDogRig --probe all
FireDogRig --probe hex-left
FireDogRig --set hex-left.ip_address=192.168.0.11 --save
FireDogRig --config ../../config/hardware.json --probe all
FireDogRig --timeout 40 --log debug --probe all
FireDogRig --repl                        # typed prompt instead of the menu
```

`--set` edits are applied first, then `--save`, then the actions — so
`--set ... --save --list` shows the saved state. An explicit `--config` path is
resolved against the caller's working directory before the rig moves to its own.

## Where the code lives

| File | Role |
|---|---|
| `tools/rig/main.cpp` | arguments, working-directory normalization, front-end choice, exit code |
| `tools/rig/RigMenu.{h,cpp}` | the numbered menu — turns selections into command strings |
| `tools/rig/RigConsole.{h,cpp}` | the command table — menu, REPL and one-shot flags share it |
| `tools/rig/RigSession.{h,cpp}` | owns `HardwareManager` + `Workflow`; connect, wait, teardown |
| `tools/rig/RigProbe.{h,cpp}` | the read-back report |
| `tools/rig/RigReport.{h,cpp}` | the plain-text configuration report (one function) |
| `tools/rig/RigConfigEdit.{h,cpp}` | the field table over `HardwareConfig` + save dispatch |

The rig links `firedog_hw` (see `docs/build-and-sdks.md`) — `src/core`,
`src/hardware`, `src/workflow`. It deliberately does **not** link `src/command`
or `src/session`: running recipes or Lua programs headless is a separate feature
and would pull in Lua, sol2 and the UI-answered blocking prompts.
