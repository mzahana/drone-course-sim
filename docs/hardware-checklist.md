# Hardware checklist — X500 V2 · SIYI A8 mini · RPi 5 / Jetson Orin Nano

**Audience:** whoever has hands on the aircraft. Instructor-led on Day 1 (inspection) and Day 4
(bench then flight).

Work through it in order and out loud. **One person reads, one person does.** If a line cannot be
ticked, the aircraft does not go to the next section — there is no "we will check that in the
air".

The flight-day roles, abort criteria and site procedure are in
[`field-procedure.md`](field-procedure.md). This document is the aircraft; that one is the
operation.

---

## 0. Standing rules

1. **Props off for every bench test.** No exceptions, no "it is only for a second". Every bench
   procedure below assumes props are off, and the props go on at exactly one point in section 5.
2. **Battery is the last thing connected and the first thing disconnected.** An unpowered aircraft
   cannot hurt anybody.
3. **The person holding the RC transmitter is the pilot in command**, and they are the only person
   who touches the sticks.
4. **Announce every state change out loud**: "connecting battery", "arming", "props on",
   "clear to arm", "disarming". The point is that everyone else can object before it happens.
5. **If anything is unexpected, disarm and disconnect the battery.** Diagnose on the bench,
   never in the air, never with power on and props fitted.

---

## 1. Kit check — before anything is powered

- [ ] Airframe: X500 V2, all four arms secure, no cracks at the arm clamps or the motor mounts.
- [ ] Four motors turn freely by hand, no grinding, no play in the bell.
- [ ] Prop nuts/adapters present and undamaged; **props NOT fitted yet**.
- [ ] Props: matched set, correct sizes, **two CW and two CCW**, no nicks or delamination.
      A chipped prop is discarded, not "used for the bench".
- [ ] Spare set of props in the kit.
- [ ] Flight controller mounted solidly, vibration mount intact, orientation arrow forward.
- [ ] GPS/compass mast upright, tight, and its arrow forward.
- [ ] SIYI A8 mini mounted, gimbal free to move through its full travel without fouling a leg,
      a cable or the landing gear.
- [ ] A8 mini Ethernet and power cables secured, with strain relief — **not** hanging on the
      connector.
- [ ] Onboard computer (Raspberry Pi 5 or Jetson Orin Nano) secured, heatsink/fan clear of
      obstruction, its cable to the flight controller secured.
- [ ] Telemetry/RC receiver antennas positioned clear of carbon and not touching each other.
- [ ] All batteries charged, none puffed, none physically damaged.
- [ ] Battery straps present and not frayed.
- [ ] RC transmitter charged, on the right model profile, throttle stick at zero.
- [ ] Laptop charged and ready, QGroundControl installed and able to connect.
- [ ] Fire-safe battery bag, first-aid kit, and the tool roll are present.

> **TODO:** insert the specific battery chemistry, cell count, capacity and C-rating in use, and
> the resulting storage/charged voltages, once the flight batteries are fixed. The voltage
> thresholds in section 2 are written for a 4S LiPo and must be corrected if the pack differs.

---

## 2. Battery discipline

This is the part that most often goes wrong and the only part that can start a fire.

**Before use**

- [ ] Pack is at a **charged** voltage, measured with a checker, not assumed from the charger.
      For a 4S LiPo: **16.6–16.8 V** total, cells within **0.05 V** of each other.
- [ ] Pack is not puffed, not warm, and has no damaged wires or connectors.
- [ ] Pack temperature is ambient. A pack straight off a charger is not flown.

**In use**

- [ ] Land at **20 % remaining**, not at the low-battery warning, and not "one more pass".
- [ ] Under load, **land immediately** if any cell reaches **3.5 V**.
- [ ] Never fully discharge. A pack taken below 3.0 V/cell is retired, not recharged.

**After use**

- [ ] Let the pack cool before charging. Never charge a warm pack.
- [ ] Charge and store in the fire-safe bag, on a non-flammable surface, attended.
- [ ] Packs not flying again today go to **storage charge** (3.8 V/cell, ~15.2 V for 4S).
- [ ] Any pack that took a hard landing is quarantined for 24 h, out of the bag, on concrete,
      before it is charged.
- [ ] Log each pack's cycle. A pack whose voltage sags noticeably more than its peers is done.

**Rule:** the battery is connected **last**, after every other check in sections 3 and 4 has
passed, and disconnected **first** the moment anything is wrong.

---

## 3. Bench — props OFF

Everything in this section happens with the aircraft on a bench, **props removed**, and someone
with a hand on the battery connector.

### 3.1 Power-up

- [ ] Say "connecting battery". Connect.
- [ ] Listen: expected ESC startup tones, all four, in unison. A single ESC out of time is a fault.
- [ ] No smoke, no smell, nothing warm. If in doubt, disconnect.
- [ ] Flight controller LED reaches its normal state.

### 3.2 Autopilot and QGroundControl

- [ ] QGroundControl connects and shows the vehicle.
- [ ] Firmware version is the one the course expects. **PX4 v1.17** matches the simulation.
- [ ] No red items in the QGC vehicle setup pages.
- [ ] Sensors calibrated: accelerometer, gyro, **compass**, level horizon. A compass calibrated
      indoors or near a car is not calibrated.
- [ ] Radio calibrated; every stick and switch moves the expected bar in QGC.
- [ ] Flight-mode switch assignments confirmed and read out loud: which position is Position
      mode, which is Altitude/Stabilized, which is **Return**, and which is the **kill** switch.
- [ ] Battery failsafe thresholds set and understood.
- [ ] RC-loss and data-link-loss actions set — see `field-procedure.md` §6.
- [ ] **`COM_RC_IN_MODE` is 0** and **`NAV_DLL_ACT` is not 0.** The simulation airframe sets
      `COM_RC_IN_MODE 4` and `NAV_DLL_ACT 0` so that a headless SITL session with no transmitter
      and no ground station can arm. On a real aircraft the first means **the pilot cannot take
      over** and the second disables the data-link failsafe. Read both back and say the values
      out loud. If anyone has been copying parameters from a simulation session, this is where it
      is caught.
- [ ] Geofence parameters set and read out loud. See `field-procedure.md` for the values.
- [ ] Home position sets when the aircraft gets a fix.

### 3.3 Motors and direction — props still off

- [ ] Using QGC's motor test at low output, spin each motor **one at a time**.
- [ ] Confirm each motor is the one QGC says it is (position 1–4 on the airframe diagram).
- [ ] Confirm each motor's **direction of rotation** matches the airframe diagram.
- [ ] Nothing rubs, rattles or buzzes.

### 3.4 Onboard computer and payload

- [ ] Onboard computer boots; you can reach it over its normal link
      (**TODO:** state the actual access method and address — Ethernet, hotspot, or serial console).
- [ ] The link to the flight controller is up. MAVROS reports `connected: true`, mode and battery
      voltage match what QGC shows.
- [ ] Clocks: the onboard computer and the autopilot agree on time. On hardware,
      **`use_sim_time` must be `false`** — the opposite of every simulation session this week.
      Getting this wrong makes every TF lookup silently wrong, exactly as in SITL.
- [ ] A8 mini reachable; the SIYI driver starts without errors.
- [ ] Gimbal responds to an angle command, moves smoothly, and returns measured attitude.
- [ ] Gimbal travel checked to the limits used in the course: **pitch −90° to +25°, yaw ±135°.**
      It must reach **90° straight down** — the geolocation maths depends on it.
- [ ] Video stream arrives, is in focus, and is not upside down.
- [ ] Detector runs on the live stream and produces detections on a test target on the ground.
- [ ] The TF chain resolves on the real aircraft, and the camera optical frame is the one the
      detections carry.
- [ ] Nothing in the student code subscribes to anything that only exists in simulation — there
      is **no ground truth** on the real aircraft.

### 3.5 Arming check — props still off

This is a rehearsal of exactly what will happen on the field, with the aircraft unable to fly.

- [ ] Everybody knows props are off and the aircraft is on the bench.
- [ ] Say "arming, props off". Arm from the transmitter.
- [ ] It arms **only** when commanded, and the motors idle at the expected low output.
- [ ] Disarm from the transmitter. It disarms **immediately**.
- [ ] Test the **kill switch** while armed. Motors stop instantly. This is the one control that
      must work; test it every single session.
- [ ] Attempt to arm in the course's flight mode and confirm PX4 refuses when it should — e.g.
      before a position fix. Read the refusal reason in QGC and recognise it.
- [ ] Disconnect the battery. Say "battery out".

### 3.6 If it arms unexpectedly

An aircraft that arms without being commanded is a fault, and it is treated as one.

1. **Shout "ARMED".** Everyone steps back and away from the prop disc — sideways, not backwards
   over your own feet.
2. **Kill switch.** Do not reach for the aircraft first.
3. If the kill switch does not stop it, **pull the battery** — approach from behind the arm, never
   across the prop arc, and only if it is on the bench and stationary. If it is moving, let it run
   out and keep everybody clear.
4. **Ground the aircraft.** It does not fly again this session.
5. Write down exactly what happened, in what order, and download the flight log before the next
   power-up. An unexplained arm is a grounding fault, not an anecdote.

Most common real causes, in order: a transmitter switch left in the armed position from the last
session, an RC channel mapped to an arming function by accident, and a stale arming command from
an onboard node that is still publishing. All three are found on the bench, which is why this
section exists.

---

## 4. Pre-flight — at the site, props still off

- [ ] `field-procedure.md` site brief has been delivered and roles assigned.
- [ ] Weather within limits, wind measured and within limits.
- [ ] Take-off area clear: at least **5 m radius**, flat, nobody inside it.
- [ ] Onlookers and students behind the briefed line.
- [ ] Airframe re-checked after transport: arms, motor mounts, gimbal, cables, connectors.
      **Transport loosens things.** This check is not a repeat, it is the point.
- [ ] Fresh, checked battery fitted and strapped. Strap tight; aircraft can be lifted by it.
- [ ] Centre of gravity correct with the gimbal and onboard computer fitted.
- [ ] RC transmitter on **first**, correct model, throttle at zero, all switches in the safe
      position — confirmed visually by a second person.
- [ ] Compass away from vehicles, rebar and steel furniture.

---

## 5. Props on — and nothing else happens while they go on

- [ ] Aircraft is **disarmed** and the battery is **disconnected**. Confirm by saying it.
- [ ] Fit props: CW and CCW in the positions the airframe diagram shows. Wrong-direction props
      is the single most common cause of an instant flip on take-off.
- [ ] Each prop nut torqued, and each prop checked for free rotation with no wobble.
- [ ] Nobody is inside the 5 m circle except the person fitting props.
- [ ] Step back, then connect the battery. Say "battery in, props on, aircraft is live".

From this moment the aircraft is treated as if it may spin up at any time.

---

## 6. "Ready to fly" — what it actually means

The aircraft is ready to fly when **all** of these are true, checked in this order, and announced:

1. **Position is valid.** GPS fix with the expected satellite count and HDOP, EKF2 converged, QGC
   shows no position warnings and the vehicle does not drift on the map.
   (**TODO:** set and state the minimum satellite count and HDOP for the site.)
2. **Home is set**, and it is where the aircraft actually is.
3. **Battery is at flight charge**, measured, and the failsafe thresholds are live.
4. **All pre-arm checks pass** — QGC shows "Ready to Fly", not a yellow or red banner. A banner
   you have learned to ignore is a banner that will one day be telling you something.
5. **The mode you intend to take off in is available**, and the pilot has said which one it is.
6. **The kill switch has been tested this session** (section 3.5), and the pilot's thumb knows
   where it is without looking.
7. **Geofence and altitude limits are set**, and everyone has heard the numbers.
8. **The take-off area is clear** and the safety observer has said "clear".
9. **Everyone on the team has said "ready"**, by name. Anybody may say "not ready" and that is the
   end of it until it is resolved.

"Ready to Fly" in green in QGroundControl means PX4's pre-arm checks pass. It does **not** mean
items 5–9. Those are yours.

---

## 7. Post-flight

- [ ] Disarm, then **disconnect the battery** before anybody approaches the aircraft.
- [ ] Props off before the aircraft is packed or carried.
- [ ] Battery voltage measured and written down; pack to the cool-down/storage routine in
      section 2.
- [ ] Motors and ESCs felt for heat. A motor noticeably hotter than its three siblings is a fault
      to chase before the next flight.
- [ ] Airframe inspected: arms, mounts, prop nuts, gimbal, connectors.
- [ ] **Flight log downloaded** from the autopilot, and the onboard computer's `ros2 bag` copied
      off. See `field-procedure.md` §Post-flight log capture — this is what Day 4's analysis and
      the capstone evidence are built from.
- [ ] Anything unexpected written down **now**, while it is still accurate.
