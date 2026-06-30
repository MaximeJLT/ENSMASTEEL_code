# Autonomous Mobile Robot — Perception & Real-Time Decision Stack

> Autonomous robot software built for **Eurobot (French Robotic Cup)** with the ENSMASTEEL team.
> This repository contains the perception, strategy, communication, and simulation stack — the software that turns an overhead camera and three low-level boards into an autonomous competitor.

The design prioritizes **separation of concerns**, **real-time robustness**, and **offline testability**: every layer has one job, communicates over an explicit protocol, and can be validated without the others present.

---

## 1. System Overview

The system is organized into **three decoupled layers**:

1. **Jetson** (NVIDIA compute + overhead camera) — sees the table, builds the world model, decides the strategy.
2. **Maman** (Raspberry Pi on the robot) — a pure dispatcher that routes orders between the Jetson and the low-level boards.
3. **Slave boards** (3 boards: motors, actuators, LiDAR) — physical execution and servo control.

```
┌───────────────────────────────────────────────────────────┐
│                       JETSON (compute)                    │
│                                                           │
│   Camera ─► vision_aruco ─► world_updater ─► WorldState   │
│                                              │            │
│                              JetsonStrategyRunner         │
│                                              │            │
│                                          Command          │
└───────────────────────────────────────────────┬───────────┘
                                                │ UDP/JSON 5005
                                                │ ◄──── 5006 ACK
                              ┌─────────────────▼──────────┐
                              │      MAMAN (Raspberry Pi)  │
                              │           maman.cpp        │
                              │       (pure dispatcher)    │
                              └─┬─────────┬──────────┬─────┘
                                │         │          │
                            UART          UART       UART
                            115200        115200     115200
                                │         │          │
                          ┌─────▼──┐ ┌────▼────┐ ┌───▼────┐
                          │ Motor  │ │ Actuator│ │ LiDAR  │
                          │ board  │ │ board   │ │ board  │
                          └────────┘ └─────────┘ └────────┘
```

A key design decision: **all intelligence lives on the Jetson, all servo control lives on the slave boards, and the dispatcher in between is deliberately stateless.** This keeps the hard real-time loops at the edges and the decision logic in one testable place.

### Hardware-in-the-loop simulator

`main_simu.py` is an **independent mirror** that replays the exact same strategy code with no hardware and no Jetson — enabling full strategy development, scoring, and regression testing offline:

```
sim_core.py (simulated WorldState, physics SimEngine)
      │
strategy_runner.py  →  Command
      │
sim_render.py (matplotlib real-time render)
```

---

## 2. Repository Layout

```
ensmasteel_comms_v1/
│
├── README.md                   ← this file
├── .gitignore
│
├── maman/                      ← RUNS ON THE RASPBERRY PI DURING A MATCH
│   ├── maman.cpp               ← UDP↔UART dispatcher (C++17)
│   ├── README.md               ← build + run on the Pi
│   └── PROTOCOL_UART.md        ← interface contract for the slave-board teammates
│
├── jetson/                     ← RUNS ON THE JETSON DURING A MATCH
│   ├── json_main.py            ← match entrypoint (vision → strategy → UDP)
│   ├── json_strategy.py        ← JetsonStrategyRunner, finite-state machine
│   ├── vision_aruco.py         ← camera capture, ArUco detection
│   ├── mapping.py              ← pixel → table (mm) homography
│   ├── world_state.py          ← Robot / Crate / Opponent / Zone classes
│   ├── world_init.py           ← initial load from zones.json
│   ├── world_updater.py        ← classify detections → WorldState
│   ├── zones.json              ← zone geometry (mm)
│   └── scenario.json           ← start position + strategy_plan
│
└── simu/                       ← DEV TOOLS — NOT USED DURING A MATCH
    ├── README.md               ← how to test without hardware
    ├── main_simu.py            ← pure matplotlib simulator
    ├── sim_core.py             ← simulated WorldState, SimEngine, scoring
    ├── sim_render.py           ← real-time matplotlib renderer
    ├── strategy_runner.py      ← StrategyRunner (mirror of json_strategy)
    ├── maman_fictive.py        ← Python-simulated dispatcher (legacy, still useful)
    ├── Fake_motor_card.py      ← simulates the motor board over UART
    ├── Fake_actuator_card.py   ← simulates the actuator board
    ├── Fake_lidar_card.py      ← simulates the LiDAR board
    ├── test_maman.py           ← validator for maman.cpp (no Jetson needed)
    ├── genetic.py              ← strategy optimization (future work)
    ├── test_individual.py
    └── opponent_scenario*.json ← opponent scenarios for the simulator
```

> **Deployment principle:** on the robot and on the Jetson, clone the repo and use only `maman/` or `jetson/`. The `simu/` directory is needed only for development and validation.

---

## 3. Coordinate Frame

```
        y+ (1000 mm)
        ↑
        │
-x ─────┼────── x+ (1500 mm)
        │
        ↓
       y- (-1000 mm)

Origin : center of the table
Unit   : millimeters
```

| Zone | x_mm | y_mm |
|------|------|------|
| Blue nest (our start) | 975 → 1325 | 625 → 975 |
| Yellow nest (opponent) | -1325 → -975 | 625 → 975 |
| Pantries 1–8 | see `zones.json` | see `zones.json` |
| Cursor start | 1250 | -1000 |

---

## 4. ArUco Markers

OpenCV dictionary: **DICT_4X4_50**

| Use | ID | Status |
|-----|----|----|
| Table corner TL | 21 | ok |
| Table corner TR | 23 | ok |
| Table corner BR | 22 | ok |
| Table corner BL | 20 | ok |
| Blue crate | 36 | ok rules |
| Yellow crate | 47 | ok rules |
| Empty crate | 41 | ok rules |
| Our robot | configurable | set in `world_updater.py → ROBOT_IDS` |
| Opponent | configurable | set in `world_updater.py → OPPONENT_IDS` |

The four corner markers anchor the homography that maps camera pixels to table millimeters; crate and robot markers are classified into the `WorldState` by `world_updater.py`.

---

## 5. UDP Protocol (Jetson ↔ Maman)

### Jetson → Maman, port 5005, 10 Hz

The Jetson streams the full world model plus an optional command at a fixed rate:

```json
{
  "type":       "world_state",
  "t_ms":       1234567890,
  "frame_id":   42,
  "mapping_ok": true,
  "world": {
    "robot":    {"x_mm": 1150.0, "y_mm": 200.0, "theta_rad": -1.57},
    "opponent": {"x_mm": -800.0, "y_mm": 300.0, "theta_rad": 0.0},
    "caisses":  [{"id": 36, "x_mm": 1150.0, "y_mm": 200.0, "status": "on_ground"}],
    "zones":    ["our_nest", "pantry_1"],
    "matchtime": 12.4
  },
  "command": {
    "kind": "GOTO",
    "x_mm": 1150.0,
    "y_mm": -600.0
  }
}
```

| `command.kind` | Fields | Description |
|----------------|--------|-------------|
| `GOTO`         | `x_mm`, `y_mm`              | Navigate to this point |
| `PICKUP`       | `crate_id`                 | Grab a crate (robot in position) |
| `DROP_ALL`     | `zone_name` (info only)    | Release all carried crates |
| `MOVE_CURSOR`  | `x_mm`, `y_mm`             | Push the thermometer cursor |
| `STOP`         | —                          | Immediate stop |
| `null`         | —                          | Heartbeat, no order |

### Maman → Jetson, port 5006, one ACK per received message

```json
{
  "type":     "ack",
  "frame_id": 42,
  "t_ms":     1234567891,
  "robot_state": {
    "x_mm":               1148.0,
    "y_mm":               198.0,
    "theta_rad":          -1.57,
    "action":             "going",
    "carried":            [36, 41],
    "obstacle":           false,
    "obstacle_dist_mm":   0.0,
    "obstacle_angle_rad": 0.0
  }
}
```

> **V1 protocol evolution:** `carried` (a list of IDs) replaces the earlier `carried_count`. `action_done` is no longer sent explicitly — the Jetson infers arrival from the `going → idle` transition. Three `obstacle*` fields were added to forward LiDAR data upstream.

---

## 6. UART Protocol (Maman ↔ Slave Boards)

### Common parameters

| Parameter | Value |
|---|---|
| Baud | 115200 |
| Frame | 8N1 |
| Flow control | none |
| Encoding | ASCII, lines terminated by `\n` |

### 6.1 — Motor board (`/dev/ttyUSB0`)

**Maman → board:** `GOTO <x_mm> <y_mm>\n`, `STOP\n`, `STATUS\n` (optional)
**Board → Maman:** `POS <x> <y> <theta>\n` (pushed at ≥10 Hz), `DONE\n` (GOTO reached), `ERR <message>\n`

### 6.2 — Actuator board (`/dev/ttyUSB1`)

**Maman → board:** `PICK\n` (close arms), `DROP\n` (open arms), `STATUS\n`
**Board → Maman:** `STATUS <state>\n` (state ∈ {idle, picking, dropping}), `DONE\n`, `ERR <message>\n`

### 6.3 — LiDAR board (`/dev/ttyUSB2`)

**Board → Maman only:** `OBST <dist_mm> <angle_rad>\n`, `CLEAR\n`

> The full protocol, examples, and a code skeleton for teammates implementing the boards live in `docs/PROTOCOL_UART.md`.

---

## 7. Inside maman.cpp

A **single-threaded** event loop using `poll()` over four file descriptors (one UDP socket, three serial ports):

```cpp
while (true) {
    poll(fds, 4, 50);  // UDP socket + 3 serial ports, 50 ms timeout

    // 1. Drain all pending UART lines
    while (motor_port.read_line(line))    motor.on_line(line);
    while (actuator_port.read_line(line)) actuator.on_line(line);
    while (lidar_port.read_line(line))    lidar.on_line(line);

    // 2. On a UDP packet from the Jetson:
    //    - parse the JSON
    //    - dispatch the command to the right UART board
    //    - build the ACK with the current state
    //    - send the ACK back
}
```

**The dispatcher contains no business logic** — no strategy, no trajectory planning, no decision-making. It translates JSON → UART text and back. This is a deliberate architectural choice: a stateless translation layer is trivial to reason about, cheap to test, and removes an entire class of bugs from the real-time path.

**Build:**
```bash
sudo apt install nlohmann-json3-dev
cd maman
g++ -std=c++17 -O2 maman.cpp -o maman
```

---

## 8. JetsonStrategyRunner — Finite-State Machine

The strategy runner consumes the world model and steps through a plan, one action at a time:

```
    IDLE ──►  _next_step()
            │     GOTO        → WAITING_GOTO
            │     PICKUP      → WAITING_PICKUP
            │     DROP_ALL    → WAITING_DROP
            │     MOVE_CURSOR → WAITING_CURSOR
            │
            │ (action confirmed OR vision arrival OR timeout)
            ▼
          IDLE → next step
```

**Dual-source GOTO arrival confirmation** (whichever confirms first wins):
1. `robot_state.action: going → idle` transition in the dispatcher ACK, or
2. Vision-measured distance to target below `ARRIVAL_TOL` (60 mm).

This redundancy is what makes the loop robust in practice: odometry and vision fail in different ways, so accepting either as a confirmation keeps the robot moving when one source drops out, while the timeout guarantees the FSM never deadlocks.

---

## 9. Tunable Parameters

| Parameter | File | Default | Notes |
|-----------|------|---------|-------|
| `ROBOT_IDS` | `world_updater.py` | configurable | ArUco ID of our robot |
| `OPPONENT_IDS` | `world_updater.py` | configurable | ArUco ID of the opponent |
| `ARRIVAL_TOL` | `json_strategy.py` | 60 mm | GOTO arrival distance |
| `PICKUP_RANGE` | `json_strategy.py` | 150 mm | Arm reach |
| `PICKUP_TIMEOUT_S` | `json_strategy.py` | 3.0 s | Arm-close duration |
| `DROP_TIMEOUT_S` | `json_strategy.py` | 2.5 s | Arm-open duration |
| `GOTO_TIMEOUT_S` | `json_strategy.py` | 30.0 s | Max GOTO timeout |
| `world_corners_mm` | `mapping.py` | ±1500 / ±1000 | Table dimensions |
| `USE_CAMERA` | `vision_aruco.py` | `True` | `False` for camera-less tests |
| `MAMAN_IP` | `json_main.py` | `127.0.0.1` | Pi address on the network |
| UART ports (maman) | `argv[1..3]` | `/dev/ttyUSB0/1/2` | pin with udev rules |

---

## 10. Validation

What has been validated end to end, without the physical robot:

- [x] Pure simulation pipeline (`main_simu.py`) — full strategy run and scored
- [x] `maman_fictive.py` — simulated Jetson↔dispatcher loop over UDP
- [x] `maman.cpp` compiles and runs on Linux
- [x] `maman.cpp` validated against three Python board fakes over `socat` virtual serial ports
- [x] UART protocol documented in `docs/PROTOCOL_UART.md`
- [x] Integration test `test_maman.py` passing:
  - heartbeat → correct ACK
  - `GOTO (1000, -500)` → progressive motion observable in the ACK stream
  - `PICKUP` → `carried` updated
  - `DROP_ALL` → `action: dropping` then back to `idle`
  - `STOP` → accepted

This is the part I care about most as an engineer: the entire decision and communication stack can be exercised and regression-tested **before** the robot is even powered on, which is what makes iteration fast and competition day predictable.

---

## 11. Running the System

### Pure simulation (no hardware, no Jetson)

```bash
cd simu
python3 main_simu.py
```

### Dispatcher alone (off-hardware validation)

```bash
# Terminals 1-3: create 3 virtual PTY pairs
socat -d -d pty,raw,echo=0,link=/tmp/tty_motor_master    pty,raw,echo=0,link=/tmp/tty_motor_slave &
socat -d -d pty,raw,echo=0,link=/tmp/tty_actuator_master pty,raw,echo=0,link=/tmp/tty_actuator_slave &
socat -d -d pty,raw,echo=0,link=/tmp/tty_lidar_master    pty,raw,echo=0,link=/tmp/tty_lidar_slave &

# Terminals 4-6: launch the fakes
cd simu
python3 Fake_motor_card.py    /tmp/tty_motor_slave
python3 Fake_actuator_card.py /tmp/tty_actuator_slave
python3 Fake_lidar_card.py    /tmp/tty_lidar_slave

# Terminal 7: dispatcher
cd maman
./maman /tmp/tty_motor_master /tmp/tty_actuator_master /tmp/tty_lidar_master

# Terminal 8: tester (simulates the Jetson)
cd simu
python3 test_maman.py
```

### Full Jetson + dispatcher pipeline (with fakes)

Same terminals 1-7, then:
```bash
# Terminal 8: Jetson without camera (USE_CAMERA=False)
cd jetson
python3 json_main.py
```

### Production (match day)

**On the Raspberry Pi (dispatcher):**
```bash
cd maman
./maman /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2
```

**On the Jetson:**
```bash
cd jetson
python3 json_main.py
```

Two machines, two commands. No simulation files required.

---

## Tech Stack

**Languages:** C++17, Python 3
**Vision:** OpenCV (ArUco detection, homography)
**Compute:** NVIDIA Jetson, Raspberry Pi
**Comms:** UDP/JSON (nlohmann/json), UART serial
**Tooling:** socat (virtual serial), matplotlib (simulation render)

## What This Project Taught Me

This was as much an exercise in **system architecture and integration** as in algorithms. The decisions I am most satisfied with are the ones that made the system *testable and robust* rather than merely *functional*: a stateless dispatcher, an explicit protocol contract shared with teammates, dual-source arrival confirmation, and a hardware-in-the-loop simulator that let the whole team validate strategy offline. Building autonomy that survives contact with reality means designing for the failure modes first — and that mindset is the main thing I take with me from this project.
