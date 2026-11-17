package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Locale;

/**
 * TurretCalibrationOpMode
 *
 * Interactive hands-on manual calibration OpMode for measuring turret encoder resolution
 * (TURRET_DEGREES_PER_TICK) and direction polarity (TURRET_ENCODER_REVERSED) without
 * calculating complex gear ratios or motor CPR by hand.
 *
 * WORKFLOW:
 * 1. D-Pad Up/Down: Select target calibration angle (360.0°, 180.0°, or 90.0°).
 * 2. Align turret with a physical mark facing straight ahead.
 * 3. Press Gamepad 1 [A] (Cross) to ZERO the encoder.
 * 4. Manually rotate the unpowered turret by hand CLOCKWISE (CW / 順時針) 360° (or 180° / 90°).
 * 5. Press Gamepad 1 [B] (Circle) to LOCK the final reading.
 * 6. Copy the printed TURRET_DEGREES_PER_TICK and TURRET_ENCODER_REVERSED into AutoTurretShooter.java!
 */
@TeleOp(name = "1. Turret 360 Manual Calibration", group = "Calibration")
public class TurretCalibrationOpMode extends LinearOpMode {

    private DcMotorEx turretMotor;

    // Calibration Target Angles (Degrees)
    private double targetAngleDeg = 360.0;

    // Calibration State
    private int startTicks = 0;
    private int lockedTicks = 0;
    private boolean isZeroed = false;
    private boolean isLocked = false;

    // Edge Detection
    private boolean lastDUp = false;
    private boolean lastDDown = false;
    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void runOpMode() {
        // Initialize Turret Motor in Unpowered Free-Spin Mode (FLOAT, zero electromagnetic braking)
        try {
            turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            turretMotor.setPower(0.0);
        } catch (Exception e) {
            turretMotor = null;
        }

        telemetry.addLine("=== 1. TURRET 360 MANUAL CALIBRATION ===");
        if (turretMotor == null) {
            telemetry.addLine("❌ ERROR: 'turretMotor' (DcMotorEx) not found in hardwareMap!");
        } else {
            telemetry.addLine("Turret set to UNPOWERED FREE-SPIN mode.");
            telemetry.addLine("Press START to begin calibration.");
        }
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // D-Pad Up/Down: Toggle Target Calibration Angle (360.0°, 180.0°, 90.0°)
            if (gamepad1.dpad_up && !lastDUp) {
                if (targetAngleDeg == 90.0) targetAngleDeg = 180.0;
                else if (targetAngleDeg == 180.0) targetAngleDeg = 360.0;
            }
            lastDUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDDown) {
                if (targetAngleDeg == 360.0) targetAngleDeg = 180.0;
                else if (targetAngleDeg == 180.0) targetAngleDeg = 90.0;
            }
            lastDDown = gamepad1.dpad_down;

            int rawCurrentTicks = (turretMotor != null) ? turretMotor.getCurrentPosition() : 0;

            // Gamepad 1 [A]: Zero Encoder Baseline
            if (gamepad1.a && !lastA) {
                if (turretMotor != null) {
                    turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    turretMotor.setPower(0.0);
                }
                startTicks = rawCurrentTicks;
                lockedTicks = rawCurrentTicks;
                isZeroed = true;
                isLocked = false;
            }
            lastA = gamepad1.a;

            // Gamepad 1 [B]: Lock / Freeze Final Reading
            if (gamepad1.b && !lastB) {
                lockedTicks = rawCurrentTicks;
                isLocked = true;
            }
            lastB = gamepad1.b;

            // Calculate Ticks & Resolution
            int activeTicks = isLocked ? lockedTicks : rawCurrentTicks;
            int deltaTicks = activeTicks - startTicks;
            int absDeltaTicks = Math.abs(deltaTicks);

            double degreesPerTick = (absDeltaTicks > 0) ? (targetAngleDeg / absDeltaTicks) : 0.0;
            double ticksPerDegree = (absDeltaTicks > 0) ? ((double) absDeltaTicks / targetAngleDeg) : 0.0;

            // Automatic Polarity Check:
            // Top-down Clockwise (turning Right) is NEGATIVE (-angle).
            // If user turned Clockwise and deltaTicks is negative (<0) -> encoder direction matches convention (REVERSED = false).
            // If user turned Clockwise and deltaTicks is positive (>0) -> encoder is inverted (REVERSED = true).
            boolean encoderReversed = (deltaTicks > 0);

            // Telemetry Output
            telemetry.addLine("========================================");
            telemetry.addLine("  1. TURRET 360 MANUAL CALIBRATION TOOL  ");
            telemetry.addLine("========================================");
            telemetry.addData("Selected Target Angle", String.format(Locale.US, "%.1f deg (Use D-Pad Up/Down)", targetAngleDeg));
            telemetry.addData("Calibration Status", !isZeroed ? "STEP 1: PRESS [A] TO ZERO" : (isLocked ? "LOCKED (PRESS [A] TO RESTART)" : "STEP 2: ROTATE CLOCKWISE BY HAND"));
            telemetry.addLine("----------------------------------------");

            telemetry.addData("Raw Baseline Ticks", startTicks);
            telemetry.addData("Current Raw Ticks", activeTicks);
            telemetry.addData("Delta Ticks Read", deltaTicks);
            telemetry.addLine("----------------------------------------");

            if (absDeltaTicks > 10) {
                telemetry.addLine(">>> CALIBRATED ENCODER OUTPUTS <<<");
                telemetry.addData("TURRET_DEGREES_PER_TICK", String.format(Locale.US, "%.8f deg/tick", degreesPerTick));
                telemetry.addData("TURRET_TICKS_PER_DEGREE", String.format(Locale.US, "%.4f ticks/deg", ticksPerDegree));
                telemetry.addData("TURRET_ENCODER_REVERSED", encoderReversed ? "true (INVERTED POLARITY)" : "false (STANDARD POLARITY)");
                telemetry.addLine("");
                telemetry.addLine("COPY & PASTE INTO AutoTurretShooter.java:");
                telemetry.addLine(String.format(Locale.US, "public static double TURRET_DEGREES_PER_TICK = %.8f;", degreesPerTick));
                telemetry.addLine(String.format(Locale.US, "public static boolean TURRET_ENCODER_REVERSED = %b;", encoderReversed));
            } else {
                telemetry.addLine("1. Align turret forward and press Gamepad 1 [A]");
                telemetry.addLine("2. Manually spin turret CLOCKWISE (CW / 順時針) " + targetAngleDeg + "° by hand.");
                telemetry.addLine("3. Press Gamepad 1 [B] to LOCK values.");
            }

            telemetry.addLine("========================================");
            telemetry.update();

            sleep(20);
        }
    }
}
