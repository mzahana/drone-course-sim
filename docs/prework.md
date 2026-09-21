# Pre-work — do this before Day 1

**Audience:** students. **Time:** about 2 hours, most of it waiting on a download.
**Deadline:** finished and verified before the first session starts.

---

## Why this is not optional

The course is 12 contact hours across 4 sessions of 3 hours. **Nothing installs during class.**
Day 1 opens with a working stack on every laptop, and the first lab (Lab 0) assumes you can
already run `course doctor` and get a clean result.

The image is about **3.4 GB to download** and about **13 GB on disk** once unpacked. On
conference wifi, with thirty people pulling at once, that is not a ten-minute job. A student who
arrives with nothing installed spends the first hour of Day 1 downloading instead of flying —
**a third of Day 1, and it is not recoverable.**

You do not need to understand Docker to do any of this. You need to run six commands and read
one output.

---

## 1. What Docker is, in one paragraph

Everything the course needs — ROS 2 Jazzy, Gazebo Harmonic, PX4 v1.17, MAVROS, YOLO11n and its
weights — is already installed inside a single downloadable file called an **image**. Docker runs
that image as a **container**: an isolated Linux system that uses your machine's CPU and GPU but
has its own installed software. Nothing from the image touches your own system, and removing the
container removes all of it. The only thing shared between the two is one folder, described in
step 5, where your own code lives.

This is why the course can guarantee that everyone's environment is identical: it is literally
the same bytes on every machine.

---

## 2. Operating system requirements

| Your OS | Story |
|---|---|
| **Ubuntu 22.04 or 24.04** | Fully supported. This is the reference platform, and Gazebo/RViz/QGroundControl windows open natively on your desktop. Use this if you have the choice. |
| Other Linux with Docker | Works. GUI windows need an X11 server, which you almost certainly already have. Otherwise use the browser desktop below. |
| **Windows 10/11** | Works **via WSL2 with an Ubuntu 24.04 distribution**, not via Docker Desktop's Windows containers. Install WSL2, install Ubuntu 24.04 into it, then follow the Linux instructions *inside* WSL2. For the GUI, use the **browser desktop** (section 6a) — it works the same on every machine and does not depend on WSLg being set up correctly. |
| **macOS** | Docker runs the image; the GUI comes from the **browser desktop** (section 6a), so nothing here blocks you. On **Apple Silicon** the image is `linux/amd64` and runs under emulation, which is slow for Gazebo — it is usable for reading and writing code, but not comfortable for a live lab. Talk to the instructor before Day 1 if this is your only machine. |

**Nobody is blocked by graphics.** If X11 does not work on your machine — Windows, macOS, a
stubborn Linux, or an SSH session — `course desktop` gives you the entire Linux desktop in a
browser tab. Section 6a.

Hardware minimum: **4 cores, 8 GB RAM, 20 GB free disk**. Comfortable: 8 cores, 16 GB RAM.

> **TODO:** confirm whether lab machines are available on site for students on Apple Silicon,
> where the constraint is emulation speed rather than graphics, and how many.

---

## 3. Install Docker

### Ubuntu / WSL2 Ubuntu

Follow the official instructions at <https://docs.docker.com/engine/install/ubuntu/> — install
**Docker Engine**, not Docker Desktop.

Then add yourself to the `docker` group so you do not need `sudo` for every command:

```bash
sudo usermod -aG docker $USER
```

**Log out and back in** (in WSL2: `wsl --shutdown` from PowerShell, then reopen the terminal).
This step is easy to skip and it is the cause of the first error most people hit.

Check it:

```bash
docker run --rm hello-world
```

If that prints a "Hello from Docker!" paragraph, Docker is working. If it says
`permission denied while trying to connect to the Docker daemon socket`, you missed the log-out.

### Windows — the extra steps first

1. In PowerShell as administrator: `wsl --install -d Ubuntu-24.04`
2. Reboot, let Ubuntu finish first-time setup, set a username and password.
3. Open the Ubuntu terminal and follow the Ubuntu instructions above **inside it**.
4. Optional: check WSLg gives you native windows with `sudo apt install -y x11-apps && xeyes`.
   A pair of eyes should appear as a Windows window. If it does not, do not spend time on it —
   use the browser desktop in section 6a, which is the supported path on Windows anyway.

---

## 4. NVIDIA GPU — only if you have one

**You do not need a GPU.** On CPU, YOLO11n runs at **22–26 ms per frame at 640 px** and **42 ms
at the course default of 960 px**, and the detector is throttled to 10 Hz either way, so the CPU
path has headroom. Every lab and the entire
capstone are designed around CPU inference. If `nvidia-smi` is not a command on your machine,
skip this whole section — you are not missing anything.

If you *do* have an NVIDIA GPU and the proprietary driver installed, install the NVIDIA Container
Toolkit so the container can see it:

<https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html>

Then:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker run --rm --gpus all ubuntu nvidia-smi
```

That last command must print your GPU. `run.sh` detects the NVIDIA runtime automatically and adds
`--gpus all` when it finds it; when it does not, it prints `[run] no NVIDIA runtime — running on
CPU` and carries on. Both are correct outcomes.

Note that the default image ships the **CPU build of PyTorch** deliberately: the CUDA wheels pull
roughly 3 GB of `cuda-toolkit`, `cudnn`, `cublas` and `triton` that most laptops cannot use. A
CUDA image is a separate tag, not the default.

---

## 5. Get the course environment

```bash
git clone <repo-url> && cd drone-course-sim
./install.sh
```

> **TODO:** insert the clone URL.
>
> **TODO:** `install.sh` pulls `ghcr.io/mzahana/drone-course-sim:jazzy` by default. **That image
> is not published yet.** Publish it before the cohort is told to run this, or tell students to
> set `IMAGE=` to wherever it does live. If the pull fails the script says so and prints the
> local-build command instead of leaving you guessing.

`install.sh` prints a line for each thing it does:

1. checks that Docker is installed **and usable by your user** (this is where a missed
   `usermod -aG docker` is caught);
2. creates the shared workspace at `~/drone_course_shared_volume/ros2_ws/src`;
3. pulls the image unless it is already present, and tags it locally as `drone-course-sim:jazzy`
   — which is the name `run.sh` looks for;
4. warns you if there is less than 20 GB free where Docker keeps its images.

The pull is the long part: **about 3.4 GB down, about 13 GB on disk after unpacking.** Start it
on a connection you trust and leave it. Check you have the space first:

```bash
df -h ~
```

### The shared folder — where your code lives

| Where | Path |
|---|---|
| On your own machine (host) | `~/drone_course_shared_volume/ros2_ws/src` |
| Inside the container | `~/shared_volume/ros2_ws/src` |

They are the same folder. **Edit your code on the host with your normal editor; build and run it
inside the container.** Nothing you write there is lost when the container is deleted. Anything
you write *outside* that folder inside the container **is** lost — so do not put your work in
`/tmp` or your container home directory.

---

## 6. Start the container

```bash
./run.sh
```

`run.sh` starts the container the first time and re-enters it every time after, so it is the one
command you use all week. It prints what it decided:

```
[run] no NVIDIA runtime — running on CPU
[run] gz partition 'course_<yourname>', ROS_DOMAIN_ID 37
```

Both lines are normal. The partition and domain ID exist because Gazebo's discovery is **not**
scoped by `ROS_DOMAIN_ID` and travels by multicast: without them, on a shared classroom network,
one student's PX4 can find another student's Gazebo and spawn its aircraft into somebody else's
world. Do not set `USE_HOST_NETWORK=1`, and do not "fix" your networking by doing so.

To leave the container, type `exit`. Your simulation stops; your files do not.

---

## 6a. If you have no graphical desktop — the browser desktop

Gazebo, RViz and QGroundControl need somewhere to draw. On Ubuntu that is your own desktop and
`run.sh` wires it up for you. Everywhere else — Windows, macOS, an SSH session, or a Linux
machine whose X11 refuses to cooperate — use the browser desktop instead. It is not a lesser
path; it is a complete Linux desktop, and everything in the course runs on it.

Inside the container:

```bash
course desktop
```

It prints a line telling you to open **<http://localhost:6080/vnc.html>** in your browser. Do
that, and you get a desktop you can put Gazebo, RViz and QGroundControl windows on.

Two things to know:

- **In the terminal where you ran it**, do what it tells you: `export DISPLAY=:99`. Terminals you
  open *after* that pick it up on their own.
- `run.sh` publishes port 6080 on `127.0.0.1` only, so the desktop is reachable from your own
  machine and from nowhere else. Change `VNC_PORT` if something else on your machine already uses
  6080.

Check it works before Day 1:

```bash
course desktop
export DISPLAY=:99
course qgc        # QGroundControl should appear in the browser tab
```

---

## 7. Run `course doctor`

Inside the container:

```bash
course doctor
```

This is the single check that decides whether your pre-work is done. **A healthy run looks like
this** — the exact version strings and the `DISPLAY` value will differ:

```
Drone course environment check

ROS 2
  ok    ROS_DISTRO=jazzy
  ok    ros2 CLI
  ok    mavros installed
  ok    ros_gz_bridge installed

Gazebo
  ok    gz 8.11.0
  ok    PX4 gz models present

PX4
  ok    px4 binary built (v1.17.0)
  ok    course gimbal airframe available
  ok    model course_x500_base installed
  ok    model course_x500 installed
  ok    model course_gimbal installed
  ok    model x500_course_gimbal installed
  ok    model course_target_vehicle installed
  ok    course world installed

Geoid data (MAVROS will not start without it)
  ok    GeographicLib geoids

Perception
  ok    ultralytics
  ok    yolo11n weights baked in
  warn  no CUDA — inference will run on CPU (fine, just slower)

Ground station
  ok    QGroundControl v4.4.4
  ok    web desktop available (course desktop)

Exercises
  ok    skeleton lab3_offboard
  ok    skeleton lab4_perception
  ok    skeleton lab5_follow
  ok    skeleton capstone_follow

Graphics
  ok    DISPLAY=:0

Your workspace
  ok    /home/user/shared_volume/ros2_ws/src

Environment looks good.
```

Read it like this:

- **`ok`** — fine, move on.
- **`warn  no CUDA`** — expected on any machine without an NVIDIA GPU. **Not a problem.**
- **`warn  no DISPLAY`** — no Gazebo, RViz or QGroundControl windows. Either start `run.sh` from
  a graphical desktop, or use the browser desktop (section 6a). Sort this out before Day 1.
- **`warn  no workspace yet`** — run `course init`.
- **Any `FAIL`** — stop and fix it. `course doctor` exits non-zero and prints
  `Something is wrong above.` Take the failing line to `troubleshooting.md`, which is indexed by
  the message you are looking at.

The exact output above was produced by:

```bash
docker run --rm drone-course-sim:jazzy course doctor
```

which is also a quick way to check the image itself without starting a container you have to
clean up afterwards. Run that way there is no display and no shared volume, so the two `warn`
lines for `DISPLAY` and the workspace are expected.

---

## 8. Prove the simulation actually runs

Two terminals. In the first:

```bash
./run.sh          # from the host, in the repo directory
course sim
```

Wait for PX4 to finish starting — you are looking for the Gazebo window and, in the terminal,
`INFO [commander] Ready for takeoff!`. Cold start takes under a minute.

In a second host terminal:

```bash
./run.sh          # re-enters the SAME container
course bringup
```

That starts MAVROS, the Gazebo bridges, the detector and the TF tree. Then, in a third:

```bash
./run.sh
course verify     # asserts map -> camera_optical_frame is correct
course test       # 8 end-to-end checks: camera, detector, target, blackout
```

`course test` is the honest check. A good run ends with eight `ok` lines, including
`detector FINDS the target vehicle`. If `course verify` and `course test` both pass, your machine
is ready for every lab in the course.

Run `course test` with `tier:=-1` if you want it to be deterministic — that leaves the target's
route idle so the test drives the vehicle itself.

### The rest of the `course` commands

You will meet these during the week; there is nothing to do with them now beyond knowing they
exist. `course` on its own prints the list.

| Command | What it does |
|---|---|
| `course doctor` | Check the environment. Start here whenever anything looks wrong. |
| `course init` | Create your workspace in the shared volume. |
| `course sim` | PX4 SITL + Gazebo with the course aircraft. |
| `course bringup` | MAVROS, the Gazebo bridges, the detector and the TF tree. |
| `course verify` | Assert the full `map -> base_link -> camera_optical_frame` chain. |
| `course test` | The 8 end-to-end checks. |
| `course score` | Grade a running mission against the simulator's ground truth. |
| `course new lab3` | Copy a lab skeleton into your workspace. **It never overwrites your work.** |
| `course solution 3` | Install the reference solution *alongside* yours, as a separate package, so both build and you can compare behaviour. |
| `course desktop` | The browser desktop (section 6a). |
| `course qgc` | QGroundControl. It finds SITL by itself on UDP 14550 — give it a few seconds. |
| `course log` | Copy the newest PX4 flight log out of PX4's build tree into `~/shared_volume/logs/`, which is the only way to reach it from your host. Day 2's log analysis depends on this. |
| `course rviz` | RViz with the course layout already set up. |
| `course mavros` | MAVROS alone against a running SITL. |

---

## 9. Optional but recommended refresher

If your Linux, Python or ROS 2 are rusty, an hour here is well spent. Day 3 moves fast.

- Shell basics: paths, `cd`, tab completion, environment variables, running things in the
  background.
- Python: classes, `numpy` array arithmetic, list comprehensions.
- ROS 2: the official Jazzy beginner tutorials, "CLI tools" and "Client libraries". Being able to
  read `ros2 topic list`, `ros2 topic echo`, `ros2 topic hz` and `ros2 node info` without looking
  them up will save you real time in Lab 3 and Lab 4.

---

## Checklist

Tick every line before Day 1. If any line is unticked, mail the instructor **before** the
session, not on the morning of it.

- [ ] My OS is on the supported list (or I have spoken to the instructor).
- [ ] At least 20 GB of free disk space.
- [ ] `docker run --rm hello-world` works **without `sudo`**.
- [ ] (NVIDIA GPU only) `docker run --rm --gpus all ubuntu nvidia-smi` prints my GPU.
- [ ] I cloned the repo and ran `./install.sh` to completion.
- [ ] The image is pulled — `docker image ls` shows it.
- [ ] `~/drone_course_shared_volume/ros2_ws/src` exists on my machine.
- [ ] `./run.sh` drops me into a container prompt.
- [ ] `course doctor` prints **`Environment looks good.`** with no `FAIL` lines.
- [ ] I can see GUI windows — either `course doctor` shows `ok DISPLAY=...`, **or**
      `course desktop` gives me a desktop at <http://localhost:6080/vnc.html>.
- [ ] `course qgc` opens QGroundControl and it connects to SITL.
- [ ] `course sim` brings up Gazebo and PX4 reaches `Ready for takeoff!`.
- [ ] `course bringup` runs in a second terminal without errors.
- [ ] `course verify` passes.
- [ ] `course test` reports 8 `ok` checks.
- [ ] I know which folder on my own machine is the shared workspace, and I know that work saved
      anywhere else inside the container is lost.
