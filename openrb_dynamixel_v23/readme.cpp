/*
===============================================================================
                           PROJECT OVERVIEW: Rubik Solver / DXL Controller
===============================================================================

This project controls a multi-servo vertical arm (Arm1, Arm2, Gripper, etc.)
based on Dynamixel XL-430 and compatible servos using an Arduino-compatible
board (e.g., OpenRB-150 or GIGA R1 WiFi).  The firmware is modularized into
five key subsystems for maintainability and reliability.

Each module mirrors the original monolithic code — the logic and timing remain
EXACTLY identical — only reorganized for readability and reuse.

-------------------------------------------------------------------------------
                           FILE STRUCTURE & DEPENDENCIES
-------------------------------------------------------------------------------

  openrb_dynamixel_v16.ino
      ├── main entry point (setup + loop)
      ├── includes all other headers
      ├── owns global vars, Serial I/O, and main control flow
      └── calls process_serial_command() to parse commands

  ┌── utils.h / utils.cpp
  │     ├── shared math & helper routines
  │     │     • clamp, rad2deg, deg2rad
  │     │     • serial_printf_verbose() template for formatted Serial output
  │     │     • pvToTicksPerSec(), estimateTravelTimeMs()
  │     │     • mapf(), constrainf() for float-safe range mapping
  │     └── included by all other modules
  │
  ├── servos.h / servos.cpp
  │     ├── defines ServoConfig class (id, limits, direction, zero ticks)
  │     ├── initializes all servo instances (arm1, arm2, grip, etc.)
  │     ├── provides helpers:
  │     │     • find_servo(), ticks2deg(), deg2ticks()
  │     │     • init_servo_limits()
  │     │     • torqueOnGroup(), ledOnGroup(), etc.
  │     │     • checkStall(), isMoving(), getPos_deg()
  │     └── owns the global `dxl` Dynamixel2Arduino instance
  │
  ├── vertical_kinematics.h / vertical_kinematics.cpp
  │     ├── encapsulates the geometric arm model
  │     │     • Forward + inverse kinematics
  │     │     • solve_deg_angles_from_xy()
  │     │     • update_from_angles()
  │     │     • gripper alignment logic
  │     ├── provides global instance `kin`
  │     └── shared constants: `l_mm`
  │
  ├── movement.h / movement.cpp
  │     ├── all coordinated motion control
  │     │     • cmdMoveSmooth()     – smooth single-servo move
  │     │     • cmdMoveXSyncSmooth(), cmdMoveYSyncSmooth() – dual-arm sync
  │     │     • cmdMoveSmoothXYG()  – X/Y/G coordinated motion
  │     │     • cmdMoveSmoothG()    – gripper-only motion
  │     │     • cmdMoveXmm(), cmdMoveYmm() – high-level move commands in mm
  │     │     • cmdTestMove(), cmdTestMoveX() – diagnostic patterns
  │     ├── uses both `kin` (geometry) and `dxl` (hardware)
  │     └── heavily uses utils (timing, math, printf)
  │
  ├── cmd_parser.h / cmd_parser.cpp
  │     ├── contains the serial command interpreter
  │     │     • process_serial_command()
  │     │     • arduino_sscanf() lightweight sscanf
  │     │     • print_servo_status(), cmdInfo(), cmdSetLimit()
  │     ├── performs text parsing for console commands
  │     └── bridges user input to the motion layer (movement.cpp)
  │
  └── readme.cpp (this file)
        ├── purely documentation (no code compiled)
        └── safe to leave in Arduino project folder

-------------------------------------------------------------------------------
                           PROJECT EXECUTION FLOW
-------------------------------------------------------------------------------

1.  **setup()**
      - Initializes Serial and Dynamixel bus.
      - Pings all known servos.
      - Sets default position mode.
      - Reads and enforces servo limits via `init_servo_limits()`.
      - Prints available commands.

2.  **loop()**
      - Waits for a newline-terminated serial command.
      - Passes it to `process_serial_command(line)`.

3.  **process_serial_command()**
      - Parses text-based console commands like:
            MOVE 11 2300
            MOVEDEG 11 45
            MOVEX 10
            MOVEY 60
            TESTMOVE 12
      - Invokes motion routines from movement.cpp.
      - Displays results and diagnostic info.

4.  **movement.cpp**
      - Performs smooth acceleration/deceleration profiles.
      - Manages coordinated multi-servo motion and final “nudge” corrections.
      - Communicates with hardware through `dxl` and geometry via `kin`.

5.  **vertical_kinematics.cpp**
      - Converts desired X/Y/G positions into joint angles (inverse kinematics).
      - Converts servo angles back to X/Y (forward kinematics) for feedback.

6.  **servos.cpp**
      - Low-level interface to Dynamixel hardware.
      - Ensures servos remain within safe limits.
      - Provides convenience groups for torque and LED control.

7.  **utils.cpp**
      - Provides reusable math, timing, and logging functions shared everywhere.

-------------------------------------------------------------------------------
                           BUILDING IN ARDUINO IDE
-------------------------------------------------------------------------------

- Place all `.cpp` and `.h` files in the same Arduino sketch folder.
- Ensure the main file remains named **openrb_dynamixel_v16.ino**.
- Requires the **Dynamixel2Arduino** library.
- Recommended baud rate: 57600.
- Compile for a board that supports hardware Serial (OpenRB-150, GIGA R1, etc.).
- All other modules are compiled automatically by the Arduino IDE.

-------------------------------------------------------------------------------
                           SERIAL CONSOLE COMMANDS
-------------------------------------------------------------------------------

  VERBOSEON / VERBOSEOFF
      Toggle console logging verbosity.

  MOVE <id> <absgoal|±rel>
      Move one servo to absolute or relative tick value.

  MOVEDEG <id> <angle_deg>
      Move one servo to a given angle in degrees.

  MOVEX / MOVEY <float mm>
      Perform coordinated motion along X or Y axis (uses kinematics).

  MOVEXDEG <a1> <a2> <aw>
      Move Arm1, Arm2, and wrist synchronously by tick deltas.

  TESTMOVE <id> [cycles]
      Perform diagnostic micro-moves to assess repeatability.

  INFO <id>
      Print servo control table parameters (mode, limits, velocity).

  LEDON / LEDOFF <id>
      Toggle servo LED for quick diagnostics.

  MOVECENTER
      Move all servos to midpoint (TICK_ZERO).

-------------------------------------------------------------------------------
                           DEBUGGING NOTES
-------------------------------------------------------------------------------

- If a servo fails to respond, ensure correct ID and wiring (TTL bus).
- `verboseOn` allows detailed motion and error tracing.
- All motion functions detect and stop on stall events
  (`checkStall()` using current and temperature thresholds).
- Coordinate calculations rely on `l_mm`; adjust as needed
  for your specific arm geometry.

-------------------------------------------------------------------------------
                           END OF FILE
-------------------------------------------------------------------------------
*/
