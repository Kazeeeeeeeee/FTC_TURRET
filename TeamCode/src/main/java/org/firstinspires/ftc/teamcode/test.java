package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

/**
 * test.java
 *
 * BasicOpMode testing file illustrating how to call and use the single
 * high-level turret auto-aim function in your match startup file.
 */
@TeleOp(name = "test.java BasicOpMode", group = "Test")
public class test extends OpMode {

    // 1. Instantiate the Turret Subsystem
    private final TurretSubsystem turret = new TurretSubsystem();

    @Override
    public void init() {
        // 2. Configure REV Hub IMU Logo and USB directions
        turret.logoFacingDir = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        turret.usbFacingDir  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // 3. Configure Basket Coordinates & Tuned Power Multiplier
        turret.basketXInches = 48.0;  // Basket X from Step 1 calibration
        turret.basketYInches = 48.0;  // Basket Y from Step 1 calibration
        turret.basketHeightInches = 43.0;
        turret.powerMultiplier = 1.0; // Tuned multiplier from Step 3

        // 4. Initialize Hardware & Home Turret facing DIRECTLY FORWARD
        turret.init(hardwareMap);

        telemetry.addLine("=== test.java INITIALIZED ===");
        telemetry.addLine("Turret initialized facing DIRECTLY FORWARD.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // =========================================================================
        // HIGH-LEVEL SINGLE AUTO-AIM FUNCTION CALL
        // =========================================================================
        // Automatically updates IMU, Odometry, turns Servo, and spins Flywheel Motor continuously!
        boolean feedTrigger = gamepad1.right_trigger > 0.5 || gamepad1.b;
        TurretAimCalculator.AimResult aim = turret.updateAutoAim(feedTrigger);

        // Press Gamepad 1 [A] to reset starting pose to (0, 0, 0)
        if (gamepad1.a) {
            turret.resetOdometryAndIMU();
        }

        // Output Telemetry
        telemetry.addLine("========================================");
        telemetry.addLine("           test.java BasicOpMode        ");
        telemetry.addLine("========================================");
        telemetry.addData("Aim Lock Status", aim.statusMessage);
        telemetry.addData("Distance to Basket", String.format(Locale.US, "%.2f in", aim.horizontalDistanceInches));
        telemetry.addData("Turret Relative Angle", String.format(Locale.US, "%.1f deg", aim.turretRelativeAngleDegrees));
        telemetry.addData("Turret Servo Position", String.format(Locale.US, "%.3f", aim.servoPosition));
        telemetry.addData("Calculated Flywheel RPM", String.format(Locale.US, "%.0f RPM", aim.requiredFlywheelRPM));
        telemetry.addData("Continuous Motor Power", String.format(Locale.US, "%.1f%%", aim.motorPower * 100.0));
        telemetry.addLine("========================================");
        telemetry.update();
    }
}
