package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/**
 * TurretAutoAimTeleOp
 *
 * PRODUCTION STARTUP FILE for FTC Competition matches.
 * Uses TurretSubsystem for continuous flywheel rotation, IMU rotation tracking,
 * goBILDA Pinpoint 2D odometry, and fire-gating state machine.
 */
@TeleOp(name = "Turret Auto Aim Startup", group = "TeleOp")
public class TurretAutoAimTeleOp extends OpMode {

    // Subsystem Instance
    private final TurretSubsystem turret = new TurretSubsystem();

    // Drive Motors
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private boolean hasDriveMotors = false;

    // Edge Detection for Profile Selection in init_loop
    private boolean lastDLeft = false;
    private boolean lastDRight = false;

    @Override
    public void init() {
        // 1. Set Alliance Selection (RED or BLUE)
        turret.alliance = Alliance.RED;

        // 2. Set Start Pose from PoseStorage (if saved from Autonomous) or Default Starting Pose
        if (PoseStorage.hasSavedPose) {
            turret.setStartPose(
                    PoseStorage.currentPose.getX(DistanceUnit.INCH),
                    PoseStorage.currentPose.getY(DistanceUnit.INCH),
                    PoseStorage.currentPose.getHeading(AngleUnit.DEGREES)
            );
        } else {
            TurretFieldConfig.StartProfile profile = TurretFieldConfig.getSelectedProfile();
            turret.setStartPose(0.0, 0.0, profile.startHeadingDeg);
        }

        // 3. Initialize Subsystem Hardware
        turret.init(hardwareMap);

        // 4. Initialize Optional Mecanum Drive Motors
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

        telemetry.addLine("=== TURRET MATCH STARTUP INITIALIZED ===");
        telemetry.addData("Alliance", turret.alliance);
        telemetry.addData("Health Status", turret.isHealthy() ? "HEALTHY" : "UNHEALTHY (SEE ERRORS)");
        if (!turret.initErrors.isEmpty()) {
            for (String err : turret.initErrors) {
                telemetry.addData("INIT ERROR", err);
            }
        }
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // D-Pad Left/Right: Cycle Start Profile
        if (gamepad1.dpad_left && !lastDLeft) {
            TurretFieldConfig.SELECTED_PROFILE_INDEX--;
            if (TurretFieldConfig.SELECTED_PROFILE_INDEX < 0) {
                TurretFieldConfig.SELECTED_PROFILE_INDEX = TurretFieldConfig.START_PROFILES.size() - 1;
            }
        }
        lastDLeft = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !lastDRight) {
            TurretFieldConfig.SELECTED_PROFILE_INDEX++;
            if (TurretFieldConfig.SELECTED_PROFILE_INDEX >= TurretFieldConfig.START_PROFILES.size()) {
                TurretFieldConfig.SELECTED_PROFILE_INDEX = 0;
            }
        }
        lastDRight = gamepad1.dpad_right;

        // Display profile selection and UNCONFIRMED CALIBRATIONS telemetry
        turret.displayInitLoopTelemetry(telemetry);
    }

    @Override
    public void loop() {
        // Driver Fire Trigger
        boolean triggerPressed = gamepad1.right_trigger > 0.5 || gamepad1.b;

        // Single Auto-Aim & Fire Gating Function Call
        TurretAimCalculator.AimResult aim = turret.updateAutoAim(triggerPressed);

        // Driver Re-Zero Pose button combo: BACK + START (require both)
        if (gamepad1.back && gamepad1.start) {
            turret.resetToStartPose();
        }

        // Mecanum Drive Controls
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

        // =========================================================================
        // DRIVER-FACING TELEMETRY BLOCK
        // =========================================================================
        Pose2D pose = turret.getCurrentPose();

        telemetry.addLine("========================================");
        telemetry.addLine("        COMPETITION MATCH DRIVER HUD    ");
        telemetry.addLine("========================================");
        telemetry.addData("SYSTEM HEALTH", turret.isHealthy() ? "OK / HEALTHY" : "CRITICAL UNHEALTHY");
        telemetry.addData("ALLIANCE", turret.alliance);
        telemetry.addData("CAN FIRE?", turret.canFire ? "READY TO FIRE!" : "BLOCKED (" + turret.blockedReason + ")");
        telemetry.addLine("----------------------------------------");

        telemetry.addData("Robot Pose (X, Y)", String.format(Locale.US, "%.1f, %.1f in", pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH)));
        telemetry.addData("Heading (Yaw)", String.format(Locale.US, "%.1f deg", pose.getHeading(AngleUnit.DEGREES)));
        telemetry.addData("IMU Diagnostic Delta", String.format(Locale.US, "%.1f deg", turret.getImuYawDeltaDeg()));
        telemetry.addData("Distance to Basket", String.format(Locale.US, "%.1f in", aim.horizontalDistanceInches));
        telemetry.addLine("----------------------------------------");

        telemetry.addData("Target Flywheel RPM", String.format(Locale.US, "%.0f RPM", turret.getTargetFlywheelRPM()));
        telemetry.addData("Measured Flywheel RPM", String.format(Locale.US, "%.0f RPM", turret.getMeasuredFlywheelRPM()));
        telemetry.addData("Continuous Power", String.format(Locale.US, "%.1f%%", aim.motorPower * 100.0));
        telemetry.addData("Turret Angle / Pos", String.format(Locale.US, "%.1f deg (%.3f)", aim.turretRelativeAngleDegrees, aim.servoPosition));
        telemetry.addData("Feed Servo State", turret.getFeedState());
        telemetry.addLine("========================================");
        telemetry.update();
    }
}
