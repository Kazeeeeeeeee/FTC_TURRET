# FTC Turret Auto-Aim & Flywheel Shooter System Guide

This module provides a complete, camera-less **Auto-Aiming Turret & Flywheel Shooter System** for FIRST Tech Challenge (FTC) robots using the **goBILDA Pinpoint Odometry Computer**.

---

## 📁 System Architecture & File Overview

| File Name | Location | Role / Description |
|---|---|---|
| [`FindBasketPositionOpMode.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/FindBasketPositionOpMode.java) | `TeamCode/...` | **Calibration OpMode**: Finds the exact 2D $(X, Y)$ position of the basket on the field. |
| [`TurretShooterTuningTeleOp.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TurretShooterTuningTeleOp.java) | `TeamCode/...` | **Tuning OpMode**: Live FTC Dashboard tuning for flywheel motor power and efficiency. |
| [`TurretAimCalculator.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TurretAimCalculator.java) | `TeamCode/...` | **Subsystem / Math Engine**: Reusable kinematic and ballistic calculation functions. |
| [`TurretAutoAimTeleOp.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TurretAutoAimTeleOp.java) | `TeamCode/...` | **Main Production TeleOp Startup File**: Full auto-aiming and shooting during match play. |
| [`TurretAutoAimAutonomous.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TurretAutoAimAutonomous.java) | `TeamCode/...` | **Production Autonomous Sample**: Auto-aiming and shooting in autonomous modes. |

---

## 🎯 Field Coordinate System & Default Alignment

- **Field Origin $(0, 0)$**: Fixed starting location where the robot is placed when the OpMode initializes.
- **Axes**:
  - Forward movement = **$+X$**
  - Leftward movement = **$+Y$**
  - $0^\circ$ Heading = Pointing along $+X$ (Forward)
- **Starting Turret Alignment**:
  - No matter what position the turret servo was in previously, when an OpMode initializes (`init()`), the turret servo **automatically homes facing DIRECTLY FORWARD** relative to the robot body ($0^\circ$ relative angle).

---

## 🛠️ Step-by-Step Calibration & Tuning Procedure

Follow these steps in order before using the auto-aim system in actual match play.

---

### STEP 1: Calibrate 2D Basket Coordinates $(X, Y)$

1. Place your robot at your designated **fixed starting position** on the field.
2. Select and run **`Find Basket 2D Position Calibration`** (`FindBasketPositionOpMode`) on the FTC Driver Station.
3. Manually push or drive the robot until it is **positioned directly right under the center of the target basket**.
4. Press **`Gamepad 1 [A]`** (or Cross) to snapshot and lock the coordinates.
5. Record the values displayed on the Driver Station telemetry:
   - `MARKED Basket X` (e.g. `48.50 inches`)
   - `MARKED Basket Y` (e.g. `52.20 inches`)

---

### STEP 2: Input Hardware & Basket Parameters

Open `TurretShooterTuningTeleOp.java` (and `TurretAutoAimTeleOp.java`) and update the initial configuration variables:

```java
// Set Basket position from Step 1 calibration
public static double BASKET_X = 48.50; // Replace with your marked Basket X
public static double BASKET_Y = 52.20; // Replace with your marked Basket Y
public static double BASKET_HEIGHT = 43.0; // Target basket height above ground (inches)

// Set hardware dimensions
public static double FLYWHEEL_RADIUS_IN = 2.0;  // Radius of shooting wheel (inches)
public static double LAUNCH_ANGLE_DEG = 50.0;   // Launcher ramp angle relative to ground (deg)
```

---

### STEP 3: Live Flywheel Power Tuning via FTC Dashboard

1. Connect your laptop to the Robot Controller Wi-Fi network.
2. Open a web browser and navigate to **FTC Dashboard**:
   - `http://192.168.43.1:8080/dash` (Control Hub) or `http://192.168.49.1:8080/dash` (Phone)
3. On the FTC Driver Station, select and INIT **`Turret Shooting Power Tuning (FTC Dashboard)`** (`TurretShooterTuningTeleOp`).
4. Press **START** on the Driver Station.
5. **Observe Auto-Aiming**:
   - Move or drive the robot across different areas of the field.
   - The turret servo will **automatically rotate** to keep aiming directly at the basket coordinates regardless of robot movement or heading.
6. **Tune Power Multiplier on FTC Dashboard**:
   - In the right-hand panel of FTC Dashboard, find the `TurretShooterTuningTeleOp` configuration group.
   - Pull **Gamepad 1 Right Trigger** (or press **`Gamepad 1 [B]`**) to spin the flywheel and shoot a sample.
   - **Adjust `POWER_MULTIPLIER`**:
     - ❌ **Shot falls short of basket**: Increase `POWER_MULTIPLIER` (e.g., `1.05`, `1.10`, `1.15`).
     - ❌ **Shot overshoots past basket**: Decrease `POWER_MULTIPLIER` (e.g., `0.95`, `0.90`, `0.85`).
     - 💡 *(Optional)* You can also adjust `FLYWHEEL_EFFICIENCY` (default `0.65`).
7. Test shooting from **multiple positions** (close, medium, far distance).
8. Once shots consistently land inside the basket from all distances, record your final tuned `POWER_MULTIPLIER` value.

---

### STEP 4: Update Production Startup File & Run

1. Open **`TurretAutoAimTeleOp.java`** (and `TurretAutoAimAutonomous.java`).
2. Set `aimCalculator.powerMultiplier` to your tuned value:
   ```java
   aimCalculator.powerMultiplier = 1.08; // Your tuned multiplier from Step 3
   aimCalculator.basketXInches = 48.50;  // Your marked Basket X from Step 1
   aimCalculator.basketYInches = 52.20;  // Your marked Basket Y from Step 1
   ```
3. Run **`Turret Auto Aim Startup`** during competition matches.

---

## 🎮 Gamepad Controls Quick Reference

| Button | Action |
|---|---|
| **`Gamepad 1 [A]`** | Reset Odometry starting pose to `(0, 0, 0)` |
| **`Gamepad 1 [B]`** | Spin Flywheel & Shoot Sample |
| **`Gamepad 1 [X]`** | Toggle Auto-Aim Mode ON / OFF |
| **`Right Trigger`** | Analog Flywheel Spin & Feed Trigger |
| **`Left Stick (X/Y)`** | Drive Robot (Mecanum/Arcade Drive) |
| **`Right Stick (X)`** | Turn Robot |
