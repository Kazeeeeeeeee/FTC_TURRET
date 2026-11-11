# FTC Turret Auto-Aim & Flywheel Shooter System

This module provides a complete, camera-less **Auto-Aiming Turret & Continuous Flywheel Shooter System** for FIRST Tech Challenge (FTC) robots using the **REV Hub IMU** (for real-time rotation detection) and **goBILDA Pinpoint Odometry** (for 2D position tracking).

---

## 🔧 Driver Station Hardware Configuration & Code Mapping

In the **FTC Driver Station** app under `Configure Robot`, you **MUST** configure your hardware devices with the exact names listed below:

| Config Name in Driver Station | Device Category & Type in Driver Station | Recommended REV Hub Port | Code Mapping (`hardwareMap.get`) | Exact File & Line Reference |
|---|---|---|---|---|
| `imu` | **Internal IMU** (REV Hub IMU) | Built-in Control Hub / Expansion Hub | `hardwareMap.get(IMU.class, "imu")` | [`TurretSubsystem.java:74`](src/main/java/org/firstinspires/ftc/teamcode/TurretSubsystem.java#L74)<br>[`TurretAutoAimTeleOp.java:113`](src/main/java/org/firstinspires/ftc/teamcode/TurretAutoAimTeleOp.java#L113)<br>[`TurretShooterTuningTeleOp.java:81`](src/main/java/org/firstinspires/ftc/teamcode/TurretShooterTuningTeleOp.java#L81) |
| `pinpoint` | **I2C Device** -> `goBILDA Pinpoint Odometry Computer` | I2C Bus Port (Port 0-3) | `hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint")` | [`TurretSubsystem.java:84`](src/main/java/org/firstinspires/ftc/teamcode/TurretSubsystem.java#L84)<br>[`FindBasketPositionOpMode.java:44`](src/main/java/org/firstinspires/ftc/teamcode/FindBasketPositionOpMode.java#L44)<br>[`TurretAutoAimTeleOp.java:122`](src/main/java/org/firstinspires/ftc/teamcode/TurretAutoAimTeleOp.java#L122) |
| `turretServo` | **Servos** -> `Servo` (Standard Servo) | Servo Port (e.g. Servo Port 0) | `hardwareMap.get(Servo.class, "turretServo")` | [`TurretSubsystem.java:97`](src/main/java/org/firstinspires/ftc/teamcode/TurretSubsystem.java#L97)<br>[`TurretAutoAimTeleOp.java:132`](src/main/java/org/firstinspires/ftc/teamcode/TurretAutoAimTeleOp.java#L132)<br>[`TurretShooterTuningTeleOp.java:100`](src/main/java/org/firstinspires/ftc/teamcode/TurretShooterTuningTeleOp.java#L100) |
| `flywheelMotor` | **Motors** -> `REV Expansion Hub Motor` (DcMotorEx) | Motor Port (e.g. Motor Port 0) | `hardwareMap.get(DcMotorEx.class, "flywheelMotor")` | [`TurretSubsystem.java:105`](src/main/java/org/firstinspires/ftc/teamcode/TurretSubsystem.java#L105)<br>[`TurretAutoAimTeleOp.java:136`](src/main/java/org/firstinspires/ftc/teamcode/TurretAutoAimTeleOp.java#L136)<br>[`TurretShooterTuningTeleOp.java:104`](src/main/java/org/firstinspires/ftc/teamcode/TurretShooterTuningTeleOp.java#L104) |
| `feedServo` *(Optional)* | **Servos** -> `Servo` (Feed Trigger Servo) | Servo Port (e.g. Servo Port 1) | `hardwareMap.get(Servo.class, "feedServo")` | [`TurretSubsystem.java:114`](src/main/java/org/firstinspires/ftc/teamcode/TurretSubsystem.java#L114)<br>[`TurretAutoAimTeleOp.java:141`](src/main/java/org/firstinspires/ftc/teamcode/TurretAutoAimTeleOp.java#L141)<br>[`TurretShooterTuningTeleOp.java:108`](src/main/java/org/firstinspires/ftc/teamcode/TurretShooterTuningTeleOp.java#L108) |
| `leftFront` *(Optional)* | **Motors** -> `REV Expansion Hub Motor` | Motor Port 0 | `hardwareMap.get(DcMotor.class, "leftFront")` | [`TurretAutoAimTeleOp.java:148`](src/main/java/org/firstinspires/ftc/teamcode/TurretAutoAimTeleOp.java#L148) |
| `rightFront` *(Optional)* | **Motors** -> `REV Expansion Hub Motor` | Motor Port 1 | `hardwareMap.get(DcMotor.class, "rightFront")` | [`TurretAutoAimTeleOp.java:149`](src/main/java/org/firstinspires/ftc/teamcode/TurretAutoAimTeleOp.java#L149) |
| `leftBack` *(Optional)* | **Motors** -> `REV Expansion Hub Motor` | Motor Port 2 | `hardwareMap.get(DcMotor.class, "leftBack")` | [`TurretAutoAimTeleOp.java:150`](src/main/java/org/firstinspires/ftc/teamcode/TurretAutoAimTeleOp.java#L150) |
| `rightBack` *(Optional)* | **Motors** -> `REV Expansion Hub Motor` | Motor Port 3 | `hardwareMap.get(DcMotor.class, "rightBack")` | [`TurretAutoAimTeleOp.java:151`](src/main/java/org/firstinspires/ftc/teamcode/TurretAutoAimTeleOp.java#L151) |

---

## 📁 System Architecture & File Overview

| File Name | Location | Role / Description |
|---|---|---|
| [`TurretSubsystem.java`](src/main/java/org/firstinspires/ftc/teamcode/TurretSubsystem.java) | `TeamCode/...` | **Subsystem Wrapper**: Encapsulates Odometry, IMU, Turret Servo, and Flywheel Motor into a single function. |
| [`test.java`](src/main/java/org/firstinspires/ftc/teamcode/test.java) | `TeamCode/...` | **BasicOpMode Test File**: Example file calling the single `updateAutoAim()` function. |
| [`FindBasketPositionOpMode.java`](src/main/java/org/firstinspires/ftc/teamcode/FindBasketPositionOpMode.java) | `TeamCode/...` | **Calibration OpMode**: Finds the exact 2D $(X, Y)$ position of the basket on the field. |
| [`TurretShooterTuningTeleOp.java`](src/main/java/org/firstinspires/ftc/teamcode/TurretShooterTuningTeleOp.java) | `TeamCode/...` | **Tuning OpMode**: Live FTC Dashboard tuning for flywheel motor power, efficiency, and **servo angle range sweep**. |
| [`TurretAimCalculator.java`](src/main/java/org/firstinspires/ftc/teamcode/TurretAimCalculator.java) | `TeamCode/...` | **Math Engine**: Reusable kinematic and ballistic calculation functions. |
| [`TurretAutoAimTeleOp.java`](src/main/java/org/firstinspires/ftc/teamcode/TurretAutoAimTeleOp.java) | `TeamCode/...` | **Main Production TeleOp Startup File**: Match startup file using `TurretSubsystem`. |

---

## 🚀 How to Use the Single Auto-Aim Function in Any OpMode

To use the auto-aim system in your match startup file (`test.java` or custom OpMode):

```java
public class MyMatchOpMode extends OpMode {

    // 1. Instantiate TurretSubsystem
    private final TurretSubsystem turret = new TurretSubsystem();

    @Override
    public void init() {
        // 2. Set IMU orientation, Basket pose, and calibrated Servo Angle Range
        turret.logoFacingDir = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        turret.usbFacingDir  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        turret.basketXInches = 48.0;  // Marked X from Step 1
        turret.basketYInches = 48.0;  // Marked Y from Step 1
        turret.powerMultiplier = 1.08; // Tuned multiplier from Step 3
        turret.servoTotalRangeDegrees = 500.0; // Calibrated degrees per 1.0 servo position

        // 3. Initialize hardware (Turret homes facing DIRECTLY FORWARD)
        turret.init(hardwareMap);
    }

    @Override
    public void loop() {
        // 4. CALL THE SINGLE AUTO-AIM FUNCTION IN YOUR LOOP
        // Automatically updates IMU & Odometry, rotates Servo, and runs Flywheel continuously!
        boolean feedTriggerPressed = gamepad1.right_trigger > 0.5 || gamepad1.b;
        TurretAimCalculator.AimResult aim = turret.updateAutoAim(feedTriggerPressed);
    }
}
```

---

## 🛠️ Complete Step-by-Step Calibration & Tuning Procedure

### STEP 1: Calibrate 2D Basket Coordinates $(X, Y)$
1. Place your robot at your designated **fixed starting position** on the field.
2. Select and run **`Find Basket 2D Position Calibration`** (`FindBasketPositionOpMode`) on the FTC Driver Station.
3. Manually push or drive the robot until it is **positioned directly right under the center of the target basket**.
4. Press **`Gamepad 1 [A]`** to snapshot and lock the coordinates.
5. Record `MARKED Basket X` and `MARKED Basket Y`.

---

### STEP 2: Calibrate Servo Angle Range (`degreesPerServoPosition`)
Different servos have different angular sweep ranges ($180^\circ$, $270^\circ$, $360^\circ$, etc.). To calibrate the exact mechanical angle per servo position:
1. Run **`Turret Shooting Power Tuning (FTC Dashboard)`** (`TurretShooterTuningTeleOp`).
2. Press & hold **`Gamepad 1 [A]`**. The servo position will slowly increment, turning the turret leftward (+angle).
3. Watch the physical turret turn **one full circle ($360^\circ$)** or a half circle ($180^\circ$).
4. As soon as it completes 1 full circle, **release `Gamepad 1 [A]`**.
5. Telemetry will calculate and display:
   $$\text{servoTotalRangeDegrees} = \frac{360.0^\circ}{\Delta P}$$
6. Record the calculated `servoTotalRangeDegrees` value (e.g. `500.0 deg`) and input it into `SERVO_RANGE_DEGREES` on FTC Dashboard or in `turret.servoTotalRangeDegrees`.

---

### STEP 3: Live Flywheel Power Tuning via FTC Dashboard
1. Connect laptop to Robot Controller Wi-Fi network.
2. Navigate to **FTC Dashboard**: `http://192.168.43.1:8080/dash`.
3. Run **`Turret Shooting Power Tuning (FTC Dashboard)`** (`TurretShooterTuningTeleOp`).
4. Drive/move the robot across the field.
5. Adjust `POWER_MULTIPLIER` under `TurretShooterTuningTeleOp` until samples drop into the basket.

---

### STEP 4: Deploy to Match Startup File (`test.java` or `TurretAutoAimTeleOp.java`)
Set `turret.powerMultiplier` and `turret.servoTotalRangeDegrees` to your tuned values and run your OpMode!
