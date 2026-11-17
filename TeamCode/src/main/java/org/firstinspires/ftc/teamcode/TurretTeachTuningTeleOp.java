package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

/**
 * TurretTeachTuningTeleOp
 *
 * Gamepad-only teach-mode OpMode (NO LAPTOP REQUIRED).
 *
 * CONTROLS:
 * - Left/Right Sticks: Mecanum drive
 * - D-Pad Up/Down: Target RPM trim (+/-25 RPM) or Power trim (+/-0.02)
 * - D-Pad Left/Right: Turret angle trim (+/-0.5 deg)
 * - Right Trigger: Fire (gated by canFire)
 * - Gamepad [X]: Mark "Shot Scored" -> computes effective efficiency & saves to table
 * - Gamepad [Y]: Delete last point in efficiency table
 * - Gamepad [B]: Save efficiency table to settings file
 */
@TeleOp(name = "Turret Teach Tuning (Gamepad Only)", group = "Tuning")
public class TurretTeachTuningTeleOp extends OpMode {

    private final TurretSubsystem turret = new TurretSubsystem();

    // Drive Motors
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private boolean hasDriveMotors = false;

    // Voltage Sensor for CSV log
    private VoltageSensor batteryVoltageSensor;

    // Trims
    private double rpmPowerTrim = 0.0;
    private double turretAngleTrimDeg = 0.0;

    // Edge Detection Flags
    private boolean lastDUp = false;
    private boolean lastDDown = false;
    private boolean lastDLeft = false;
    private boolean lastDRight = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastB = false;

    private static final String LOG_FILE_NAME = "turret_shot_log.csv";

    @Override
    public void init() {
        turret.init(hardwareMap);

        try {
            leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            rightFront = hardwareMap.get(DcMotor.class, "rightFront");
            leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            rightBack = hardwareMap.get(DcMotor.class, "rightBack");

            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            hasDriveMotors = true;
        } catch (Exception e) {
            hasDriveMotors = false;
        }

        try {
            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            batteryVoltageSensor = null;
        }

        telemetry.addLine("=== TURRET TEACH TUNING READY ===");
        telemetry.addLine("Drive around and fire with Right Trigger.");
        telemetry.addLine("Press [X] when shot scores to record distance-efficiency point.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // D-Pad Up/Down: Trim RPM or Power
        if (gamepad1.dpad_up && !lastDUp) {
            rpmPowerTrim += turret.useClosedLoopFlywheel ? 25.0 : 0.02;
        }
        lastDUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !lastDDown) {
            rpmPowerTrim -= turret.useClosedLoopFlywheel ? 25.0 : 0.02;
        }
        lastDDown = gamepad1.dpad_down;

        // D-Pad Left/Right: Trim Turret Angle
        if (gamepad1.dpad_left && !lastDLeft) {
            turretAngleTrimDeg -= 0.5;
        }
        lastDLeft = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !lastDRight) {
            turretAngleTrimDeg += 0.5;
        }
        lastDRight = gamepad1.dpad_right;

        // Run Auto-Aim Loop
        boolean triggerPressed = gamepad1.right_trigger > 0.5;
        TurretAimCalculator.AimResult aim = turret.updateAutoAim(triggerPressed);

        Pose2D pose = turret.getCurrentPose();
        double dist = aim.horizontalDistanceInches;
        double measuredRPM = turret.getMeasuredFlywheelRPM();
        double targetRPM = aim.requiredFlywheelRPM + (turret.useClosedLoopFlywheel ? rpmPowerTrim : 0.0);

        // Apply turret angle trim
        if (turretAngleTrimDeg != 0.0) {
            aim.turretRelativeAngleDegrees += turretAngleTrimDeg;
            aim.servoPosition = turret.aimCalculator.calculateServoPosition(aim.turretRelativeAngleDegrees);
        }

        // Gamepad [X]: Shot Scored -> compute effective efficiency and insert into table
        if (gamepad1.x && !lastX) {
            double effectiveEff;
            if (measuredRPM > 100) {
                // effectiveEff = v0(d) / (measuredRPM * 2pi/60 * flywheelRadiusIn)
                double wheelEdgeSpeedInSec = (measuredRPM * (Math.PI / 30.0)) * turret.aimCalculator.flywheelRadiusInches;
                effectiveEff = aim.targetLaunchVelocityInSec / wheelEdgeSpeedInSec;
            } else {
                // Back out from commanded power / default efficiency
                effectiveEff = turret.aimCalculator.flywheelEfficiency * (1.0 + rpmPowerTrim);
            }

            turret.efficiencyTable.insertOrMergePoint(dist, effectiveEff);
            logShot(pose, dist, targetRPM, measuredRPM, turretAngleTrimDeg, true);
        }
        lastX = gamepad1.x;

        // Gamepad [Y]: Delete last point in table
        if (gamepad1.y && !lastY) {
            turret.efficiencyTable.removeLastPoint();
        }
        lastY = gamepad1.y;

        // Gamepad [B]: Save table to CSV file
        if (gamepad1.b && !lastB) {
            turret.efficiencyTable.saveToFile();
        }
        lastB = gamepad1.b;

        // Log non-scored shots when trigger pulled
        if (triggerPressed && turret.canFire) {
            logShot(pose, dist, targetRPM, measuredRPM, turretAngleTrimDeg, false);
        }

        // Mecanum Drive
        if (hasDriveMotors) {
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double lfPower = drive + strafe + turn;
            double rfPower = drive - strafe - turn;
            double lbPower = drive - strafe + turn;
            double rbPower = drive + strafe - turn;

            double max = Math.max(Math.abs(lfPower), Math.max(Math.abs(rfPower),
                         Math.max(Math.abs(lbPower), Math.abs(rbPower))));
            if (max > 1.0) {
                lfPower /= max;
                rfPower /= max;
                lbPower /= max;
                rbPower /= max;
            }

            leftFront.setPower(lfPower);
            rightFront.setPower(rfPower);
            leftBack.setPower(lbPower);
            rightBack.setPower(rbPower);
        }

        // Telemetry
        telemetry.addLine("========================================");
        telemetry.addLine("     TURRET TEACH TUNING (GAMEPAD)      ");
        telemetry.addLine("========================================");
        telemetry.addData("Can Fire?", turret.canFire ? "READY TO FIRE" : "BLOCKED (" + turret.blockedReason + ")");
        telemetry.addData("Table Size (Points)", turret.efficiencyTable.size());
        telemetry.addLine("----------------------------------------");
        telemetry.addData("Distance to Basket", String.format(Locale.US, "%.2f in", dist));
        telemetry.addData("Target vs Measured RPM", String.format(Locale.US, "%.0f / %.0f RPM", targetRPM, measuredRPM));
        telemetry.addData("RPM/Power Trim", String.format(Locale.US, "%.2f", rpmPowerTrim));
        telemetry.addData("Turret Angle Trim", String.format(Locale.US, "%.1f deg", turretAngleTrimDeg));
        telemetry.addData("Turret At Limit?", aim.turretAtLimit ? "YES" : "NO");
        telemetry.addLine("----------------------------------------");
        telemetry.addLine("Press [X] when shot scores (adds point)");
        telemetry.addLine("Press [Y] to delete last point | [B] to save table");
        telemetry.addLine("========================================");
        telemetry.update();
    }

    private void logShot(Pose2D pose, double dist, double targetRPM, double measuredRPM, double angleTrim, boolean scored) {
        try {
            double voltage = (batteryVoltageSensor != null) ? batteryVoltageSensor.getVoltage() : 0.0;
            File file = AppUtil.getInstance().getSettingsFile(LOG_FILE_NAME);
            String line = String.format(Locale.US, "%d,%.2f,%.2f,%.1f,%.2f,%.0f,%.0f,%.2f,%.1f,%b\n",
                    System.currentTimeMillis(), pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                    pose.getHeading(AngleUnit.DEGREES), dist, targetRPM, measuredRPM, voltage, angleTrim, scored);

            ReadWriteFile.writeFile(file, line);
        } catch (Exception ignored) {}
    }
}
