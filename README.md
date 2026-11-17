# FTC Turret Auto-Aim & Flywheel Shooter System Guide

This repository contains a complete, camera-less **Auto-Aiming Turret & Flywheel Shooter System** for FIRST Tech Challenge (FTC) robots using **goBILDA Pinpoint Odometry** (as the single pose source for 2D position and heading) and the **REV Hub IMU** (as an optional diagnostic).

---

## 📁 System Overview & File Map

| OpMode / File Name | Location | Role / Description |
|---|---|---|
| `TurretFieldConfig.java` | `TeamCode/...` | **Master Configuration**: Holds all measured hardware dimensions, pod offsets, start profiles, heights, and CONFIRMED flags. |
| `TurretSubsystem.java` | `TeamCode/...` | **Subsystem Wrapper**: Encapsulates single pose tracking, turret unwrapping, closed-loop flywheel, and feed state machine into `updateAutoAim()`. |
| `TurretAimCalculator.java` | `TeamCode/...` | **Math Engine**: Ballistic trajectory math, descending $v_y < 0$ check, power saturation check, and turret angle unwrapping/limits. |
| `EfficiencyTable.java` | `TeamCode/...` | **Lookup Table**: Distance-dependent flywheel efficiency table with linear interpolation and CSV file storage. |
| `Alliance.java` | `TeamCode/...` | **Alliance Enum**: Defines RED and BLUE basket positions with rotational symmetry mirroring. |
| `PoseStorage.java` | `TeamCode/...` | **Pose Handoff**: Static holder to persist end pose from Autonomous to TeleOp. |
| `InchesPerTickCalibrationOpMode.java` | `TeamCode/...` | **Step 4 OpMode**: Driver Station calibration OpMode for `INCHES_PER_TICK` push test. |
| `PinpointSanityCheckOpMode.java` | `TeamCode/...` | **Step 5 OpMode**: Spin test and push direction test diagnostic. |
| `FindBasketPositionOpMode.java` | `TeamCode/...` | **OpMode**: Direct drive-under basket position calibrator. |
| `TriangulateBasketOpMode.java` | `TeamCode/...` | **OpMode**: Least-squares ray intersection calibration to solve Basket 2D position $(X, Y)$ from 2+ spots. |
| `BasketAimCheckOpMode.java` | `TeamCode/...` | **Step 9 OpMode**: Aim check & gamepad2 nudge tool with drive enabled and shooter disabled. |
| `TurretShooterTuningTeleOp.java` | `TeamCode/...` | **Step 8 OpMode**: Two-Point Servo Calibration (`SERVO_TOTAL_RANGE_DEGREES`) & FTC Dashboard live tuning. |
| `TurretTeachTuningTeleOp.java` | `TeamCode/...` | **Step 10 OpMode**: Gamepad-only distance-efficiency table teach mode & shot CSV logging. |
| `TurretAutoAimTeleOp.java` | `TeamCode/...` | **Step 11 OpMode**: Production competition TeleOp startup file with Driver HUD. |
| `TurretAutoAimAutonomous.java` | `TeamCode/...` | **Step 11 OpMode**: Production Autonomous sample passing end pose to `PoseStorage`. |
| `test.java` | `TeamCode/...` | **Test OpMode**: Template demonstrating single `turret.updateAutoAim()` function call. |
| `Test.java` | `TeamCode/...` | **Test OpMode**: Alias template for `test.java`. |

---

## STEP 0: Driver Station Hardware Configuration Table

In the **FTC Driver Station** app under `Configure Robot`, configure hardware devices with these exact names:

| Config Name in Driver Station | Device Category & Type in Driver Station | Recommended REV Hub Port | Code Mapping (`hardwareMap.get`) |
|---|---|---|---|
| `pinpoint` | **I2C Device** $\rightarrow$ `goBILDA Pinpoint Odometry Computer` | **I2C Bus Port 1, 2, or 3** *(Do NOT use I2C Bus 0 as it is shared with internal IMU)* | `hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint")` |
| `turretServo` | **Servos** $\rightarrow$ `Servo` (Standard Servo) | Servo Port 0 | `hardwareMap.get(Servo.class, "turretServo")` |
| `flywheelMotor` | **Motors** $\rightarrow$ `REV Expansion Hub Motor` (`DcMotorEx`) | Expansion Hub Motor Port 3 | `hardwareMap.get(DcMotorEx.class, "flywheelMotor")` |
| `feedServo` *(Optional)* | **Servos** $\rightarrow$ `Servo` (Feed Trigger Servo) | Servo Port 1 | `hardwareMap.get(Servo.class, "feedServo")` |
| `imu` *(Optional Diagnostic)* | **Internal IMU** (REV Hub IMU) | Built-in Control Hub / Expansion Hub | `hardwareMap.get(IMU.class, "imu")` |
| `leftFront` *(Optional)* | **Motors** $\rightarrow$ `REV Expansion Hub Motor` | Control Hub Motor Port 0 | `hardwareMap.get(DcMotor.class, "leftFront")` |
| `rightFront` *(Optional)* | **Motors** $\rightarrow$ `REV Expansion Hub Motor` | Control Hub Motor Port 1 | `hardwareMap.get(DcMotor.class, "rightFront")` |
| `leftBack` *(Optional)* | **Motors** $\rightarrow$ `REV Expansion Hub Motor` | Control Hub Motor Port 2 | `hardwareMap.get(DcMotor.class, "leftBack")` |
| `rightBack` *(Optional)* | **Motors** $\rightarrow$ `REV Expansion Hub Motor` | Control Hub Motor Port 3 | `hardwareMap.get(DcMotor.class, "rightBack")` |

> ⚠️ **PORT CONFLICT WARNING**: Keep `flywheelMotor` on Expansion Hub Motor Port 3 separate from drive motors (`leftFront` Port 0, `rightFront` Port 1, `leftBack` Port 2, `rightBack` Port 3). Always plug Pinpoint into I2C Ports 1-3 (Port 0 is reserved for internal IMU).

---

## STEP 1: Field Coordinate Frame & Units

All distances are measured in **INCHES**, and angles in **DEGREES**.

```text
                  +X (Forward)
                       ^
                       |
                       |       (Basket: +X dx, +Y dy)
                       |              O
                       |
  +Y (Left) <----------(0,0) Origin (Start Point)
                   Robot Facing
                   Heading = 0°
```

- **Origin $(0, 0)$**: The exact Pinpoint tracking point location on the robot when placed at the start mark.
- **$+X$ Axis**: Straight forward along the robot's initial heading.
- **$+Y$ Axis**: Straight leftward (90 degrees counter-clockwise from $+X$).
- **Heading $\theta$**: Measured in degrees counter-clockwise from $+X$ ($0^\circ$ = Forward, $+90^\circ$ = Left).

---

## STEP 2: Mark and Define the Start Point

1. **Purpose**: Ensure robot starting position is 100% repeatable across matches.
2. **Tools Needed**: Masking tape, tape measure, T-square.
3. **Actions**:
   - Place a piece of tape flush against the back tile wall seam.
   - When placing the robot at `init()`, align the back side of the chassis flush against the tile wall and side against the tape mark.
   - Locate the **Pinpoint Tracking Point**: This is the geometric intersection point of your X and Y odometry pod wheels on the robot frame. This point is your $(0, 0)$ origin.
4. **Expected Result**: Robot initializes at the exact same physical coordinates every match within $\pm 0.1$ inches.

---

## STEP 3: Measure & Enter Pinpoint Pod Offsets & Directions

1. **Purpose**: Tell Pinpoint how far the odometry pods are mounted relative to the tracking point.
2. **Tools Needed**: Caliper or ruler (in millimeters).
3. **Actions**:
   - Measure `PINPOINT_X_OFFSET_MM`: distance sideways from tracking point to X (forward) pod (left = positive, right = negative).
   - Measure `PINPOINT_Y_OFFSET_MM`: distance forwards from tracking point to Y (strafe) pod (forward = positive, backward = negative).
   - Open `TurretFieldConfig.java` and enter values:
     ```java
     public static double PINPOINT_X_OFFSET_MM = -84.0; // TODO(user): MEASURE
     public static double PINPOINT_Y_OFFSET_MM = -168.0; // TODO(user): MEASURE
     public static boolean PINPOINT_OFFSETS_CONFIRMED = true;
     ```
4. **Expected Result**: Robot rotation in place does not cause position drift.

---

## STEP 4: Inches-Per-Tick Calibration

1. **Purpose**: Calibrate exact pod scale factor so Pinpoint reports true physical inches.
2. **Tools Needed**: 4 foam field tiles (96 inches total laid out along seams).
3. **Actions**:
   - Select and run **`Inches-Per-Tick Calibration`** (`InchesPerTickCalibrationOpMode`) on the Driver Station.
   - **Forward Test (X Pod)**:
     1. Place robot flush against start line. Press `[A]` to capture start ticks.
     2. Push robot straight forward exactly 96.0 inches (4 tile seams).
     3. Press `[B]` to capture end ticks.
     4. Repeat for 3 trials.
   - **Strafe Test (Y Pod)**: Repeat 3 trials pushing straight left 96.0 inches.
   - Telemetry displays recommended `INCHES_PER_TICK` mean and spread.
   - Copy the printed line into `TurretFieldConfig.java`:
     ```java
     public static double INCHES_PER_TICK = 0.00197945; // Measured value
     public static boolean INCHES_PER_TICK_CONFIRMED = true;
     ```
   - **Verification Push**: Re-zero with `[A]` and push 96 inches again. Reported X/Y should read between $95.5$ and $96.5$ inches.
5. **Formulas**:
   $$\text{inchesPerTick} = \frac{\text{PUSH\_DISTANCE\_INCHES}}{|\Delta \text{ticks}|}$$
   $$\text{correctedTicksPerUnit} = \text{oldTicksPerUnit} \times \frac{\text{reportedInches}}{\text{actualInches}}$$

---

## STEP 5: Pinpoint Sanity Checks

1. **Purpose**: Verify pod count directions and hardware health before aiming.
2. **OpMode**: Select **`Pinpoint Sanity Check`** (`PinpointSanityCheckOpMode`).
3. **Spin Test**:
   - Rotate robot 360 degrees in place.
   - **Pass Criteria**: Heading reads $360.0 \pm 2.0^\circ$, position drift stays under $0.5$ inches.
4. **Push Direction Test**:
   - Push robot FORWARD 48 inches $\rightarrow$ $X$ must increase to $+48.0 \pm 0.5$ in.
   - Push robot LEFT 48 inches $\rightarrow$ $Y$ must increase to $+48.0 \pm 0.5$ in.
5. **Expected Result**: Device Status reads `READY`, directions match $+X$ forward and $+Y$ left.

---

## STEP 6: Measure & Enter Basket Position (`basketDxInches`, `basketDyInches`)

1. **Purpose**: Measure exact distance from start point to the **CENTER OF THE BASKET OPENING**.
2. **Tools Needed**: Tape measure (in inches).
3. **Actions**:
   - Extend tape measure from your robot's start mark $(0,0)$ along tile seams.
   - Measure `basketDxInches`: distance FORWARD (+) or BACKWARD (-) to basket center.
   - Measure `basketDyInches`: distance LEFT (+) or RIGHT (-) to basket center.
   - Open `TurretFieldConfig.java` and update your `StartProfile`:
     ```java
     // EXAMPLE (Replace with your actual measured tape numbers):
     new StartProfile(
         "Red High Basket Start",
         "Flush against back wall, left alignment mark",
         0.0,   // startHeadingDeg
         48.50, // EXAMPLE: basketDxInches (forward distance to basket center)
         52.20, // EXAMPLE: basketDyInches (leftward distance to basket center)
         true   // confirmed flag
     )
     ```
4. **Expected Result**: Basket coordinates represent true field position relative to start point.

---

## STEP 7: Measure & Enter Height Dimensions

1. **Purpose**: Configure 3D trajectory height parameters.
2. **Tools Needed**: Tape measure / ruler (in inches).
3. **Actions**:
   - `RIM_HEIGHT_INCHES`: Measure from tile floor to top lip of high basket rim (default `43.0` in).
   - `TARGET_CLEARANCE_INCHES`: Target clearance above rim (default `3.5` in). Derived target height $= 43.0 + 3.5 = 46.5$ in.
   - `TURRET_EXIT_HEIGHT_INCHES`: Measure from tile floor to ball center as it leaves shooter ramp (default `12.0` in).
   - `EXIT_FORWARD_OFFSET_INCHES`: Measure horizontal forward distance from turret pivot shaft axis to ball exit point (default `0.0` in).
   - Open `TurretFieldConfig.java` and update:
     ```java
     public static double RIM_HEIGHT_INCHES = 43.0;
     public static boolean RIM_HEIGHT_CONFIRMED = true;
     public static double TARGET_CLEARANCE_INCHES = 3.5;
     public static boolean TARGET_CLEARANCE_CONFIRMED = true;
     public static double TURRET_EXIT_HEIGHT_INCHES = 12.0;
     public static boolean TURRET_EXIT_HEIGHT_CONFIRMED = true;
     public static double EXIT_FORWARD_OFFSET_INCHES = 0.0;
     public static boolean EXIT_FORWARD_OFFSET_CONFIRMED = true;
     ```

---

## STEP 8: Two-Point Servo Calibration (`servoTotalRangeDegrees`)

1. **Purpose**: Calibrate total turret degrees sweep per $1.0$ servo position.
2. **OpMode**: Select **`Turret Shooting Power Tuning (FTC Dashboard)`** (`TurretShooterTuningTeleOp`).
3. **Actions**:
   - Aim turret at known Mark A (e.g. $0^\circ$ straight forward) using D-pad $\rightarrow$ press **`Gamepad 1 [A]`** to record `posA`.
   - Aim turret at known Mark B (e.g. $90^\circ$ leftward) using D-pad $\rightarrow$ press **`Gamepad 1 [Y]`** to record `posB`.
   - Press **`Gamepad 1 [B]`** to calculate and save `SERVO_TOTAL_RANGE_DEGREES` to settings file!
4. **Formula**:
   $$\text{servoTotalRangeDegrees} = \frac{\text{angleB} - \text{angleA}}{\text{posB} - \text{posA}}$$

---

## STEP 9: Aim Verification with `BasketAimCheckOpMode`

1. **Purpose**: Test turret aiming at 3 field locations before enabling shooter motor.
2. **OpMode**: Select **`Basket Aim Check (Drive Only)`** (`BasketAimCheckOpMode`).
3. **Actions**:
   - Drive robot to Spot 1 (close), Spot 2 (medium), Spot 3 (far).
   - Observe turret: it should physically point directly at basket center at all locations.
   - If turret is slightly off, use **Gamepad 2 D-Pad** to nudge `dx` and `dy` by $\pm 0.25$ inches.
   - Press **`Gamepad 2 [X]`** to save final nudged coordinates.

---

## STEP 10: Flywheel & Shot Tuning

1. **Purpose**: Tune distance-dependent efficiency table so shots drop into basket from all distances.
2. **OpMode**: Select **`Turret Teach Tuning (Gamepad Only)`** (`TurretTeachTuningTeleOp`).
3. **Actions**:
   - Drive robot to various test distances.
   - Pull **Right Trigger** to fire.
   - Use D-Pad Up/Down to trim target RPM/power.
   - When a shot scores in basket, press **`Gamepad 1 [X]`** ("Shot Scored").
     - Automatically computes effective efficiency $E_{eff} = \frac{v_0(d)}{v_{wheel}}$ and stores point in `turret_efficiency_table.csv`.
   - Press **`Gamepad 1 [B]`** to save table to file.

---

## STEP 11: Match Setup & Handoff

1. **Autonomous**:
   - Run `Turret Auto Aim Autonomous Sample` (`TurretAutoAimAutonomous`).
   - At end of Autonomous, robot saves final pose to `PoseStorage.currentPose`.
2. **TeleOp**:
   - In `init_loop()`, driver uses **D-Pad Left/Right** to select `StartProfile`.
   - On start, TeleOp loads `PoseStorage.currentPose` if present.
   - Re-zero: Press **`Gamepad 1 [BACK] + [START]`** simultaneously to reset robot pose to start mark.

---

## STEP 12: Pre-Match Checklist

- [ ] Robot placed flush against start mark alignment tape.
- [ ] Correct `StartProfile` selected in `init_loop`.
- [ ] No `"UNCONFIRMED CALIBRATION"` warnings shown on telemetry.
- [ ] Pinpoint status reads `READY`.
- [ ] Turret servo initialized facing DIRECTLY FORWARD ($0^\circ$).

---

## STEP 13: Troubleshooting Guide

| Symptom | Likely Cause | Solution |
|---|---|---|
| **"UNCONFIRMED CALIBRATION" on telemetry** | One or more `<NAME>_CONFIRMED` flags in `TurretFieldConfig.java` are still `false`. | Complete measurement steps 3-8 and set flags to `true`. |
| **Turret not moving / Flywheel inert** | Required hardware missing or Pinpoint status not `READY`. | Check Driver Station config names (`pinpoint`, `turretServo`, `flywheelMotor`) and wiring. |
| **Turret off by constant angle** | Servo center offset or initial turret mounting alignment off. | Perform Step 8 Two-Point Servo Calibration. |
| **Aim error grows with distance** | `INCHES_PER_TICK` pod scale or `basketDxInches`/`basketDyInches` inaccurate. | Re-run Step 4 `InchesPerTickCalibrationOpMode` and Step 9 Aim Check. |
| **Turret swings across range** | Unwrapping limit set incorrectly or servo direction reversed. | Set `reverseServoDirection = true` in `TurretFieldConfig.java`. |
| **Shots fall short or overshoot** | Flywheel efficiency or motor power multiplier needs tuning. | Run Step 10 `TurretTeachTuningTeleOp` and score points across range. |

---

## 📊 Master Constants Reference Table

| Constant Name | Class / File | Unit | How Measured | README Step | `CONFIRMED` Flag Name |
|---|---|---|---|---|---|
| `INCHES_PER_TICK` | `TurretFieldConfig` | in/tick | 96-inch push test | Step 4 | `INCHES_PER_TICK_CONFIRMED` |
| `PINPOINT_X_OFFSET_MM` | `TurretFieldConfig` | mm | Caliper from tracking point | Step 3 | `PINPOINT_OFFSETS_CONFIRMED` |
| `PINPOINT_Y_OFFSET_MM` | `TurretFieldConfig` | mm | Caliper from tracking point | Step 3 | `PINPOINT_OFFSETS_CONFIRMED` |
| `basketDxInches` | `StartProfile` | in | Tape measure from start to basket | Step 6 | `profile.confirmed` |
| `basketDyInches` | `StartProfile` | in | Tape measure from start to basket | Step 6 | `profile.confirmed` |
| `RIM_HEIGHT_INCHES` | `TurretFieldConfig` | in | Tape measure tile to basket rim | Step 7 | `RIM_HEIGHT_CONFIRMED` |
| `TARGET_CLEARANCE_INCHES` | `TurretFieldConfig` | in | Aiming clearance above rim | Step 7 | `TARGET_CLEARANCE_CONFIRMED` |
| `TURRET_EXIT_HEIGHT_INCHES` | `TurretFieldConfig` | in | Tape measure tile to ball exit | Step 7 | `TURRET_EXIT_HEIGHT_CONFIRMED` |
| `EXIT_FORWARD_OFFSET_INCHES` | `TurretFieldConfig` | in | Distance exit point ahead of pivot | Step 7 | `EXIT_FORWARD_OFFSET_CONFIRMED` |
| `TURRET_OFFSET_X_INCHES` | `TurretFieldConfig` | in | Turret pivot offset from tracking point | Step 7 | `TURRET_OFFSETS_CONFIRMED` |
| `FLYWHEEL_RADIUS_INCHES` | `TurretFieldConfig` | in | Caliper shaft center to wheel edge | Step 7 | `FLYWHEEL_RADIUS_CONFIRMED` |
| `LAUNCH_ANGLE_DEGREES` | `TurretFieldConfig` | deg | Angle gauge on launch ramp | Step 7 | `LAUNCH_ANGLE_CONFIRMED` |
| `SERVO_TOTAL_RANGE_DEGREES` | `TurretFieldConfig` | deg | Two-Point Servo Calibration | Step 8 | `SERVO_RANGE_CONFIRMED` |
| `MIN_TURRET_ANGLE_DEG` | `TurretFieldConfig` | deg | Turret hard-stop measurement | Step 7 | `TURRET_LIMITS_CONFIRMED` |
| `MAX_TURRET_ANGLE_DEG` | `TurretFieldConfig` | deg | Turret hard-stop measurement | Step 7 | `TURRET_LIMITS_CONFIRMED` |
