# Course documentation

Five documents plus the build notes. Each one has a single audience — read the row that is you.

| Document | Who it is for | When you read it |
|---|---|---|
| [`prework.md`](prework.md) | **Students** | Before Day 1. Install Docker, pull the image, run `course doctor`. Ends in a checklist. |
| [`troubleshooting.md`](troubleshooting.md) | **Students, and instructors triaging** | The moment something breaks. Indexed by the error message on screen. |
| [`hardware-checklist.md`](hardware-checklist.md) | **Whoever has hands on the aircraft** | Day 1 inspection, and before every bench test and every flight. |
| [`field-procedure.md`](field-procedure.md) | **The Day 4 flight team** | Read aloud at the site brief, then followed in order. |
| [`syllabus.md`](syllabus.md) | **The fellowship programme office** | Course approval and scheduling. Non-technical reader. |
| [`spike-notes.md`](spike-notes.md) | **Whoever maintains the environment** | Before debugging anything that smells environmental. Twenty numbered gotchas and what each cost. |

Elsewhere in the repository and alongside it:

| | |
|---|---|
| [`../exercises/INTERFACES.md`](../exercises/INTERFACES.md) | **The frozen interface contract** — every topic name, message type, frame, QoS setting and gimbal limit that the labs, the reference solutions and the scoring script agree on. Students writing a node check their strings against this one **first**. |
| [`../README.md`](../README.md) | What the container is, and the design decisions behind it. |
| [`../PROGRESS.md`](../PROGRESS.md) | Build status and session handoff. **Read it first** if you are working on the environment rather than teaching from it. |
| `~/src/drone_courses/slides` | The four lecture decks, one per day. |
| `~/src/drone_courses/COURSE_PROPOSAL.md` | The course design: session plan, theory, capstone, rubric. |

---

## The short version

**Student, before the course:** `prework.md`, then `troubleshooting.md` when it goes wrong.

**Student, during a lab:** `troubleshooting.md` — search for the message you are looking at. If
there is no message at all, read
[Nothing is broken but nothing works](troubleshooting.md#nothing-is-broken-but-nothing-works):
the worst failures in this stack are silent. If your node is not talking to anything, check
`INTERFACES.md` before you check your code.

**Student with no working graphics:** `course desktop`, then
<http://localhost:6080/vnc.html>. `prework.md` §6a. This is a supported path, not a workaround.

**Instructor, on a hardware day:** `hardware-checklist.md` first, then `field-procedure.md`. Both
have `TODO:` markers for values that must be filled in before the day — site, permissions,
battery specification, and the geofence and failsafe parameters actually set on the aircraft.
**Fill them in; do not fly against a document that says TODO.** Note in particular the
simulation-only parameters (`COM_RC_IN_MODE`, `NAV_DLL_ACT`) that must never reach the real
aircraft — both checklists catch them.

**Programme office:** `syllabus.md` is the whole course in one file.
