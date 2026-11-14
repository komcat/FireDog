# Beacons — a temporary marker of where the camera looked

A **beacon** is a named, temporary record of a camera pose: "the camera was
looking *here*". A recipe plants it, moves the camera on, and later sends a
registered tool back to it. Without a beacon the only target for a tool is
`move_tool_to_camera`, which reads the camera's CURRENT pose, so the camera
would have to stay where it is.

| Command | What it does |
|---|---|
| `plant_beacon` | Drives the camera's device (`device`, default `gantry-main`) to a saved `position` through `move_to_position`, waits `settle_ms`, then stores the MEASURED X/Y/Z as beacon `name`. |
| `plant_beacon_here` | The same record without a move: waits `settle_ms` and stores where the camera stands NOW. The steps before it (a move, a detection's `center_after`) put it there. |
| `remove_beacon` | Removes `name`, or every beacon with `all`. A missing name is a no-op unless `must_exist` is set. |
| `move_tool_to_beacon` | `move_tool_to_camera` with the beacon as the camera point: same params, same refusals, same approach. |
| `grip_object_to_beacon` | the same for a REGISTERED gripped object (`grip_register_object`): the tip aims at beacon − (dx, dy, dz) so the object lands on it. `docs/gripped-object-design.md` §8. |

Code: `core/Beacon.h` (the record and its lifetime), `Workflow::SetBeacon` /
`BeaconFor` / `Beacons` / `ClearBeacons` / `ExpireBeacons` (the store),
`command/CommandsBeacon.cpp` (the three commands), `App::DrawAuxBeacons`
(`ui/windows/WndAux.cpp`).

## Rules

- **The camera moves to the plant location before planting.**
  - `plant_beacon` makes that move itself. It runs `move_to_position` through
    `RunRegistered` (`command/CommandRegistry.h`) rather than a copy, and hands the
    step's velocity, tolerance, timeout and approach params on unchanged.
  - `plant_beacon_here` leaves the move to the steps before it.
  - Either way, the beacon is the pose the device MEASURABLY reached after settling.
    It is never a commanded target or a typed number, so it can only sit where the
    camera actually went. Both commands share `PlantAtMeasuredPose`, the one writer.
- **Lifetime: `kBeaconLifeMin` (30 min) from planting, `remove_beacon`, product
  load, or app close.**
  - **Program start and DUT open KEEP a beacon.** It marks a place on the machine,
    not one part, and a program may plant in one recipe and consume in another.
  - **Product load clears it.** A different product's locations mean nothing.
  - **Memory only, never persisted.** A beacon that outlived a restart would be a
    licence to move nobody checked.
  - **An expired beacon reads as absent everywhere** (`BeaconFor` gives the
    reason). `App::Update` calls `ExpireBeacons`, which erases each one and logs it
    once.
- **Consuming a beacon is an APPROACH through a registration, nothing more.**
  - `move_tool_to_beacon` calls `PrepareRegMove` with the beacon's device as the
    expected reference. A registration taught against another device is refused.
  - It then calls `ApproachRefPoint` with `beacon + z_offset_mm`, so the verified
    gate, the rotation guard, `max_mm` and `MoveWithApproach` are the same ones
    `move_tool_to_camera` uses.
  - The final placement is still a measured delta (see
    `device-registration-design.md`).
- **The ctx keys `<name>_x/_y/_z` are display mirrors.** No move reads them; the
  store is the only channel.
- **Not a teach mover, not a parallel mover, not in the dry-run allowlist.**
  - A teach run disables all four commands.
  - They touch shared `Workflow` state, so `parallelBy` stays empty.
  - The registration moves are deliberately absent from `kDryRunCommands`.
- **The Aux window only DRAWS beacons.**
  - Each beacon planted on the device posing the picture is a PULSE
    (`CameraMark::Glyph::Pulse`): a dot, a fixed ring, and a second ring that grows
    and fades. It is the only moving mark on the picture, so it is never mistaken for
    a tip or a detection.
  - Its colour and beat show how close the tool is in Z. They come from the
    registered tool whose predicted tip (`DeviceRegTip`) is nearest the beacon in XY,
    by that tip's |dZ| from the beacon's Z:

    | Tool's distance from the beacon in Z | Colour | Beat |
    |---|---|---|
    | 0.3 mm or less | green | 127 bpm |
    | 0.3 to 1 mm | yellow | 80 bpm |
    | more than 1 mm, or no tool predictable | pink | 30 bpm |

  - The label, on a dark pill, is `<name>  <N>m  <tool> <height µm>`: N is the
    minutes left, and the height is the tool above the beacon. Display only; nothing
    acts on it.
  - It is projected through the current gantry pose by `DrawWorldMark`, so it stays
    on its spot as the camera moves. Off the picture it becomes an edge arrow with
    the distance.
  - There is no remove control on the window. Removing a beacon is a recipe step.
