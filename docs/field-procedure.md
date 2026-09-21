# Field procedure — Day 4 supervised flight

**Hand this to the team on the day.** It is a procedure, not an essay. Read it aloud at the site
brief; follow it in order.

**Aircraft:** X500 V2 · SIYI A8 mini · Raspberry Pi 5 or Jetson Orin Nano
**Task:** detect a ground target, keep it centred with the gimbal, geolocate it, and follow it at
a fixed standoff and altitude — the same code the students ran in SITL.

The aircraft checks are in [`hardware-checklist.md`](hardware-checklist.md). Do those first. This
document starts once the aircraft has passed section 3 (bench, props off).

> **TODO:** fill in before the day — site name and coordinates, date and time window, the
> airspace/permit reference and who holds it, the nearest hospital and its drive time, the local
> emergency number, and the names against each role below.

---

## 1. Roles

Three roles. They are filled by **three different people**, named at the brief, and they do not
swap during a flight.

### Pilot in command (PIC) — **TODO: name**

- Holds the RC transmitter and is the only person who touches the sticks.
- Has the final word on whether the aircraft flies, and on everything it does while it is flying.
- Keeps **eyes on the aircraft**, not on a screen, for the entire flight.
- Announces: "arming", "taking off", "airborne", "returning", "landing", "disarmed".
- Takes manual control the instant anything is wrong, without asking.

### Safety observer (SO) — **TODO: name**

- Watches the **airspace and the ground**, not the aircraft: people approaching, vehicles, other
  aircraft, animals, the weather changing.
- Owns the take-off/landing area and says "clear" or "not clear".
- Keeps students and onlookers behind the line.
- **Can call an abort, at any time, for any reason, without explaining first.**

### Operator — **TODO: name** (a student, supervised)

- Runs the laptop and the onboard software: starts the nodes, watches the telemetry, starts and
  stops the autonomous task.
- Calls out what the software is doing: "offboard engaged", "target acquired", "target lost",
  "estimate stale".
- Never touches the RC transmitter.
- **Can call an abort.**

Everyone else is a spectator and stays behind the line.

---

## 2. Site brief — read this out, every session

1. **Site, boundaries and the line.** Where the aircraft will fly, where the take-off area is,
   and where everybody stands. Point at them.
2. **Today's task**, in one sentence, and how long it should take.
3. **The three roles**, by name. Everyone says their own role back.
4. **Limits**: maximum altitude, geofence radius, wind limit, battery landing point.
5. **Abort word.** The word is **"ABORT"**, said loudly. Nothing else. Not "stop", not "whoa".
6. **Who can say it:** PIC, safety observer, operator — anyone, no justification needed, no
   argument afterwards.
7. **What happens on abort** (section 5).
8. **Emergency**: if the aircraft comes down among people, the PIC kills the motors; nobody
   approaches until the PIC says the battery is out. Medical kit is **TODO: location**. Fire bag
   is **TODO: location**.
9. **Questions now**, not in the air.

**Weather limits** — if any is exceeded, the flight does not happen:

| Limit | Value |
|---|---|
| Wind, steady | ≤ 7 m/s |
| Wind, gusts | ≤ 10 m/s |
| Rain | none, at all |
| Visibility | aircraft visible at the full geofence radius, unaided |
| Temperature | **TODO:** set for the site and the battery in use |

---

## 3. Limits configured on the aircraft

Read these out at the brief. If a value on the aircraft does not match this table, the table is
wrong or the aircraft is wrong — resolve it on the ground.

| Limit | Value | Where it is enforced |
|---|---|---|
| Maximum altitude AGL | **30 m** | PX4 geofence (`GF_MAX_VER_DIST`) |
| Geofence radius from home | **60 m** | PX4 geofence (`GF_MAX_HOR_DIST`) |
| Geofence action | **Hold** (PIC then recovers manually) | `GF_ACTION` |
| Altitude floor for the autonomous task | **10 m AGL** | the student mission node's clamp |
| Commanded standoff behind the target | **18 m** | the student guidance node |
| Commanded follow altitude | **12 m AGL** | the student guidance node |
| Maximum commanded ground speed | **5 m/s** | the student guidance node's clamp |
| Minimum distance from any person | **30 m** | procedure — the flight plan, not a parameter |

> **TODO:** these are the values the course is written to; **set and verify the PX4 `GF_*`
> parameters on the actual aircraft and record the values here before the day.**
>
> The standoff and altitude match what `course score` grades against in simulation: **18 m
> standoff, 12 m altitude, 8 m on-station tolerance**, so a team flies the field with the same
> numbers they tuned against. The geofence radius and the altitude floor are the two that
> deliberately do **not** match the simulator's 120 m and 5 m: the field is smaller than the
> world, and the ground is real.

Two rules that are not parameters:

- **The aircraft never flies over a person.** Not at altitude, not "just passing".
- **The ground target is driven or carried by somebody who has been briefed**, and who knows the
  drone will be above and behind them. The target person or vehicle stays inside the geofence.

---

## 4. Sequence

### 4.1 Before power

- [ ] `hardware-checklist.md` sections 1–3 complete, **bench test passed, props off**.
- [ ] Site brief delivered (section 2), roles assigned, everybody has said their role back.
- [ ] Limits (section 3) read out and confirmed against the aircraft's parameters.
- [ ] **Simulation-only parameters confirmed ABSENT.** The SITL airframe sets
      `COM_RC_IN_MODE 4` (stick input disabled) and `NAV_DLL_ACT 0` (no data-link-loss action) so
      that a headless simulation can arm with no transmitter and no ground station. **Neither may
      ever be on the real aircraft**: the first one means the pilot cannot take over, and the
      second disables the data-link failsafe. Read both back in QGroundControl and say the values
      out loud. `COM_RC_IN_MODE` must be **0**, and `NAV_DLL_ACT` must be whatever section 6 says
      it is.
- [ ] Laptop ready, telemetry link tested, logging ready to start.

### 4.2 Set-up

- [ ] `hardware-checklist.md` section 4 (pre-flight) complete.
- [ ] `hardware-checklist.md` section 5 — **props on**, battery connected, aircraft announced live.
- [ ] Everybody behind the line except the PIC and the SO.

### 4.3 Manual confidence hop — always, before any autonomous flight

No autonomous flight ever happens on the first take-off of the day.

- [ ] PIC: "clear to arm?" SO: "clear."
- [ ] Arm. Hover at about **3 m** in Position mode.
- [ ] Check: stable hover, no drift, no unusual noise, no twitch on any axis.
- [ ] Pitch, roll and yaw gently. The aircraft responds correctly and stops when the stick does.
- [ ] Check the telemetry: battery current draw sensible, all four motor outputs similar, no EKF
      warnings in QGC.
- [ ] Land, disarm, announce "disarmed".

**If anything felt wrong, the day is a bench day.** That is a normal outcome and not a failure.

### 4.4 Autonomous run

- [ ] Fresh, checked battery. Repeat the "ready to fly" list — `hardware-checklist.md` §6.
- [ ] Operator starts the onboard nodes and confirms, out loud: detector running, gimbal
      responding, TF resolving, MAVROS connected, **`use_sim_time` false**.
- [ ] Operator confirms the guidance node is commanding **18 m standoff at 12 m AGL**, the same
      numbers the team tuned in simulation, and that no node subscribes to a `ground_truth`
      topic — those exist only in the simulator.
- [ ] Target person/vehicle in position, briefed, inside the fence.
- [ ] PIC arms and takes off **manually** to the working altitude in Position mode.
- [ ] PIC stabilises the hover, then announces: "handing over, offboard on my switch".
- [ ] Operator confirms a setpoint stream is already flowing. **The stream comes first, the mode
      change second** — PX4 will refuse OFFBOARD otherwise, and drop out of it if the stream stops
      for about half a second.
- [ ] PIC switches to the offboard mode and announces "offboard engaged". **Thumb stays on the
      mode switch for the rest of the flight.**
- [ ] Operator calls the state machine out loud as it moves: SEARCH → FOLLOW → LOST → RTL.
- [ ] SO watches the aircraft and the ground. PIC watches the aircraft.
- [ ] Target moves **slowly** on the first run — walking pace, in a straight line. Speed and turns
      come on later runs, and only if the earlier ones were clean.
- [ ] Land at **20 % battery**, manually, in Position mode. Not on the failsafe.
- [ ] Disarm, announce "disarmed", disconnect the battery.

### 4.5 Between runs

- [ ] Battery swapped and logged; the used pack goes to cool down.
- [ ] Log downloaded **before** the next run (section 7) — a log you meant to download later is a
      log you lost.
- [ ] Anything unexpected discussed and resolved before the next take-off.

---

## 5. Abort

**Anyone may call an abort. The word is "ABORT". Nobody argues, nobody asks why, nobody
second-guesses it afterwards.**

### Abort criteria — call it if you see any of these

Aircraft:
- Any unexpected motion: a twitch, a drop, a yaw the operator did not command.
- Unusual sound, visible vibration, or a wobble that does not settle.
- Drifting when it should be holding position.
- Any EKF, GPS or compass warning in QGC.
- Battery below the landing threshold, or a voltage sag under load.

Software:
- The aircraft is not doing what the operator's call-out says it is doing.
- Target estimate frozen, jumping, or outside the geofence.
- Commanded speed or altitude approaching a clamp.
- Telemetry link intermittent.

Site:
- Any person, vehicle or animal crossing the line or approaching the flight area.
- Another aircraft, anywhere.
- Wind or visibility changing.
- Anybody losing sight of the aircraft.

Team:
- Anybody unsure what is happening. **Confusion is an abort criterion.**

### What happens on abort — in this order

1. **PIC switches to Position mode** — the mode switch, immediately. This takes the student code
   out of the loop and puts the aircraft under manual control. It is the first action in every
   abort, whatever the cause.
2. **PIC stops the aircraft and holds a stable hover.**
3. **Operator stops the autonomous task** and stops commanding setpoints.
4. **PIC lands**, at the take-off point if it is safe to reach it, straight down where it is if it
   is not.
5. **Disarm. Battery out. Nobody approaches until the PIC says the battery is out.**
6. Debrief before the next take-off. Write down what happened.

### Emergency — beyond an abort

- **Loss of control, aircraft heading towards people:** PIC uses the **kill switch**. A falling
  aircraft with stopped props is a much smaller problem than a flying one that is not under
  control. This decision belongs to the PIC alone and is never criticised afterwards.
- **Aircraft down:** nobody approaches until the PIC says so. Approach from the side, never across
  a prop arc. Disconnect the battery. If a battery is damaged, warm or smoking, it goes into the
  fire bag on a non-flammable surface and is watched for 15 minutes; nobody carries it indoors.
- **Fly-away:** PIC attempts Return, then the kill switch while the aircraft is still over open
  ground and within the site. Note the last known position and heading.
- **Injury:** medical kit is **TODO: location**. Call **TODO: local emergency number**. Nobody
  moves the aircraft until the injured person is dealt with.

---

## 6. RC loss and link loss

Read these out at the brief so nobody is surprised by an aircraft that starts flying itself.

| Event | What PX4 does | What the team does |
|---|---|---|
| **RC loss** (transmitter off, out of range, interference) | Failsafe triggers after the configured delay → **Return** to home, then land | SO clears the landing area and says so. PIC keeps eyes on the aircraft and regains control the moment the link returns. Nobody chases it. |
| **Data-link loss** (telemetry/laptop) | No action by itself — RC still has control | Operator says "link lost". **PIC takes manual control and lands.** Do not continue an autonomous run you cannot see the state of. |
| **Offboard setpoint stream stops** (onboard computer crash, node dies, network drop) | PX4 drops out of OFFBOARD after about **0.5 s** and falls back to its failsafe mode | This is a designed, safe behaviour. PIC takes manual control and lands. |
| **GPS / position loss** | Position-requiring modes become unavailable; PX4 falls back | PIC switches to Altitude mode and lands manually, into wind, by eye. |
| **Low battery** | Warning, then the configured action | Land manually before either. Do not fly to the failsafe. |
| **Geofence breach** | **Hold** at the boundary | PIC takes manual control and flies back inside, then lands. The run is over; investigate on the ground. |

> **TODO:** confirm and record the actual `NAV_RCL_ACT`, `NAV_DLL_ACT`, `COM_RC_LOSS_T` and the
> battery failsafe parameters set on the aircraft, and correct this table to match. **The table
> must describe the aircraft, not the intention.**
>
> **Do not carry the simulation's values across.** The SITL airframe sets `COM_RC_IN_MODE 4` and
> `NAV_DLL_ACT 0` purely so a headless simulation with no transmitter and no ground station can
> arm. On the real aircraft those two settings would delete the top two rows of this table.

Two things everyone must know by heart:

- **The mode switch is the abort.** Position mode takes the software out of the loop instantly.
- **The kill switch is the last resort**, and it is the PIC's alone.

---

## 7. Post-flight log capture

Do this **between runs**, not at the end of the day. A log you meant to download later is a log
you do not have, and the Day 4 analysis and the capstone evidence are built from these.

### From the autopilot

1. Power the aircraft on the bench with **props off**, connect QGroundControl.
2. **Analyze Tools → Log Download**, download the ULog for the run.
3. File it as `YYYY-MM-DD_run<N>_<pilot>_<task>.ulg`.
4. Open it in Flight Review (<https://review.px4.io>) — the same tool and the same workflow as
   Day 2's log lab, where `course log` pulls the SITL log out of PX4's build tree for you — and
   look at, at minimum:
   attitude and rate tracking, vibration, actuator outputs, battery voltage and current under
   load, and the mode-change timeline against the abort you called.

### From the onboard computer

Record a bag for the whole run, started before take-off and stopped after landing:

```bash
ros2 bag record -o run<N>_YYYY-MM-DD \
    /tf /tf_static \
    /camera/camera_info \
    /detections \
    /gimbal/attitude /gimbal/saturated \
    /gimbal/cmd/angle /gimbal/cmd/rate \
    /target/estimate /target/track \
    /mission/state \
    /mavros/state /mavros/local_position/pose /mavros/local_position/velocity_local \
    /mavros/setpoint_raw/local
```

Note what is **not** in that list: `/camera/image_raw` at 1280×720 and ~25 Hz will fill a disk
during a flight. Record `/detector/image_annotated` instead if you need imagery, and only for the
run where you need it. Check free space before every run.

Note also what does not exist on the real aircraft: **there is no `/target/ground_truth` and no
`/drone/ground_truth`.** Those are simulation-only scoring signals. Field performance is assessed
against the video, the estimate's own covariance, and — if a survey point is available — a fixed
marker on the ground.

### Immediately after the last run

- [ ] All ULogs downloaded and copied off the laptop to a second place.
- [ ] All bags copied off and their sizes checked (a bag that stopped early is a common surprise).
- [ ] Battery log completed; packs to storage charge.
- [ ] Debrief, written, the same day: what was flown, what was seen, what was aborted and why,
      what to change. Ten minutes while it is accurate beats an hour a week later.
