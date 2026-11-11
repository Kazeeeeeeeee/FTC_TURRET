package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/**
 * TurretShooterTuningTeleOp
 *
 * Dedicated Tuning TeleOp OpMode for:
 * 1. Flywheel shooting motor power and efficiency live tuning via FTC Dashboard.
 * 2. Servo Angle Calibration (press & hold Gamepad 1 [A] to slowly sweep turret leftward
 *    1 full circle / 360 degrees, then release to calculate exact servoTotalRangeDegrees).
 */
@Config
@TeleOp(name = "Turret Shooting Power Tuning (FTC Dashboard)", group = "Tuning")
public class TurretShooterTuningTeleOp extends OpMode {

    // =========================================================================
    // REV HUB IMU MOUNTING CONFIGURATION
    // =========================================================================
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;

    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // =========================================================================
    // FTC DASHBOARD LIVE TUNING VARIABLES (Adjustable live at http://192.168.43.1:8080/dash)
    // =========================================================================
    public static double POWER_MULTIPLIER = 1.0;       // Scale motor power
    public static double FLYWHEEL_EFFICIENCY = 0.65;   // Energy transfer efficiency (0.50 to 0.85)
    public static double BASKET_X = 48.0;              // Basket X coordinate (from calibration)
    public static double BASKET_Y = 48.0;              // Basket Y coordinate (from calibration)
    public static double BASKET_HEIGHT = 43.0;         // Basket height in inches
    public static double LAUNCH_ANGLE_DEG = 50.0;      // Launcher ramp angle in degrees
    public static double FLYWHEEL_RADIUS_IN = 2.0;     // Flywheel wheel radius in inches
    public static double SERVO_RANGE_DEGREES = 180.0;  // Total angular sweep of servo for 0.0 to 1.0
    public static boolean CONTINUOUS_FLYWHEEL = true;  // Continuous flywheel rotation at target RPM

    // =========================================================================
    // HARDWARE DEVICES
    // =========================================================================
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;
    private Servo turretServo;
    private DcMotorEx flywheelMotor;
    private Servo feedServo;

    // Optional Drive Motors
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private boolean hasDriveMotors = false;

    // Subsystem Calculator
    private final TurretAimCalculator aimCalculator = new TurretAimCalculator();
    private MultipleTelemetry dashTelemetry;

    // Servo Sweep Calibration variables
    private double calibrationServoPos = 0.5;
    private double sweepStartPos = 0.5;
    private boolean isSweeping = false;
    private static final double SWEEP_SPEED = 0.05; // Position increment per second

    private static final double FEED_REST_POSITION = 0.0;
    private static final double FEED_SHOOT_POSITION = 0.5;

    @Override
    public void init() {
        // Set up combined telemetry (Driver Station + FTC Dashboard)
        dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize REV Hub IMU
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot orientationOnRobot =
                    new RevHubOrientationOnRobot(LOGO_FACING_DIR, USB_FACING_DIR);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw();
        } catch (Exception e) {
            imu = null;
        }

        // Initialize goBILDA Pinpoint Odometry Computer
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        // Initialize Turret Servo - Set to face DIRECTLY FORWARD relative to robot
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        turretServo.setPosition(aimCalculator.getForwardServoPosition());
        calibrationServoPos = aimCalculator.getForwardServoPosition();
        sweepStartPos = aimCalculator.getForwardServoPosition();

        // Initialize Flywheel Motor
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        try {
            feedServo = hardwareMap.get(Servo.class, "feedServo");
            feedServo.setPosition(FEED_REST_POSITION);
        } catch (Exception e) {
            feedServo = null;
        }

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

        dashTelemetry.addLine("=== SHOOTER POWER & SERVO CALIBRATION READY ===");
        dashTelemetry.addLine("1. Open FTC Dashboard at http://192.168.43.1:8080/dash");
        dashTelemetry.addLine("2. Hold [A] to slowly turn servo leftward 1 full circle (360 deg) for Servo Angle Calibration.");
        dashTelemetry.update();
    }

    @Override
    public void loop() {
        // Update live parameters from FTC Dashboard @Config fields
        aimCalculator.basketXInches = BASKET_X;
        aimCalculator.basketYInches = BASKET_Y;
        aimCalculator.basketHeightInches = BASKET_HEIGHT;
        aimCalculator.launchAngleDegrees = LAUNCH_ANGLE_DEG;
        aimCalculator.flywheelRadiusInches = FLYWHEEL_RADIUS_IN;
        aimCalculator.flywheelEfficiency = FLYWHEEL_EFFICIENCY;
        aimCalculator.powerMultiplier = POWER_MULTIPLIER;
        aimCalculator.servoTotalRangeDegrees = SERVO_RANGE_DEGREES;

        // 1. Update Odometry & IMU rotation angle
        pinpoint.update();
        Pose2D currentPose = pinpoint.getPosition();

        double robotHeadingDeg;
        if (imu != null) {
            robotHeadingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        } else {
            robotHeadingDeg = currentPose.getHeading(AngleUnit.DEGREES);
        }

        Pose2D poseWithIMU = new Pose2D(
                DistanceUnit.INCH,
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                AngleUnit.DEGREES,
                robotHeadingDeg
        );

        // 2. SERVO ANGLE RANGE CALIBRATION (Hold Gamepad 1 [A] to slowly sweep turret leftward)
        if (gamepad1.a) {
            if (!isSweeping) {
                isSweeping = true;
                sweepStartPos = calibrationServoPos;
            }
            // Slowly increase servo position (turning leftward)
            calibrationServoPos += SWEEP_SPEED * 0.02; // ~0.05 pos/sec assuming 50Hz loop
            if (calibrationServoPos > 1.0) calibrationServoPos = 1.0;

            turretServo.setPosition(calibrationServoPos);
        } else {
            isSweeping = false;
        }

        // Reset calibration sweep with [X] button
        if (gamepad1.x) {
            calibrationServoPos = aimCalculator.getForwardServoPosition();
            sweepStartPos = aimCalculator.getForwardServoPosition();
            turretServo.setPosition(calibrationServoPos);
        }

        // 3. AUTO-AIM MODE (Active when NOT holding [A] for calibration sweep)
        TurretAimCalculator.AimResult aim = aimCalculator.calculateAim(poseWithIMU);
        if (!gamepad1.a) {
            turretServo.setPosition(aim.servoPosition);
            calibrationServoPos = aim.servoPosition;
        }

        // 4. Flywheel motor power control
        double triggerValue = gamepad1.right_trigger;
        boolean shootPressed = gamepad1.b || triggerValue > 0.1;

        if (CONTINUOUS_FLYWHEEL) {
            if (aim.targetInReach) {
                flywheelMotor.setPower(aim.motorPower);
            } else {
                flywheelMotor.setPower(0.0);
            }
        } else {
            if (shootPressed && aim.targetInReach) {
                flywheelMotor.setPower(aim.motorPower);
            } else {
                flywheelMotor.setPower(0.0);
            }
        }

        // 5. Feed servo actuation
        if (feedServo != null) {
            if (gamepad1.b || triggerValue > 0.7) {
                feedServo.setPosition(FEED_SHOOT_POSITION);
            } else {
                feedServo.setPosition(FEED_REST_POSITION);
            }
        }

        // 6. Manual drive (if configured)
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

        // 7. Telemetry Output (both Driver Station and FTC Dashboard)
        dashTelemetry.addLine("========================================");
        dashTelemetry.addLine("      FTC DASHBOARD & SERVO TUNING      ");
        dashTelemetry.addLine("========================================");

        // SERVO CALIBRATION DISPLAY
        double deltaPos = calibrationServoPos - sweepStartPos;
        double calcRange360 = (deltaPos > 0.01) ? (360.0 / deltaPos) : 0.0;
        double calcRange180 = (deltaPos > 0.01) ? (180.0 / deltaPos) : 0.0;

        dashTelemetry.addLine("--- SERVO ANGLE CALIBRATION ---");
        dashTelemetry.addLine("Hold [A] to slowly turn turret leftward 1 full circle (360 deg).");
        dashTelemetry.addData("Start Servo Pos", String.format(Locale.US, "%.3f", sweepStartPos));
        dashTelemetry.addData("Current Servo Pos", String.format(Locale.US, "%.3f", calibrationServoPos));
        dashTelemetry.addData("Position Delta (ΔP)", String.format(Locale.US, "%.3f", deltaPos));
        if (deltaPos > 0.01) {
            dashTelemetry.addData("--> Calculated servoTotalRangeDegrees (for 360° turn)", String.format(Locale.US, "%.1f deg", calcRange360));
            dashTelemetry.addData("--> Calculated servoTotalRangeDegrees (for 180° turn)", String.format(Locale.US, "%.1f deg", calcRange180));
        }

        dashTelemetry.addLine("----------------------------------------");
        dashTelemetry.addData("Live POWER_MULTIPLIER", String.format(Locale.US, "%.3f", POWER_MULTIPLIER));
        dashTelemetry.addData("Live SERVO_RANGE_DEGREES", String.format(Locale.US, "%.1f deg", SERVO_RANGE_DEGREES));
        dashTelemetry.addData("Lock Status", aim.statusMessage);
        dashTelemetry.addData("Robot Pose X", String.format(Locale.US, "%.2f in", currentPose.getX(DistanceUnit.INCH)));
        dashTelemetry.addData("Robot Pose Y", String.format(Locale.US, "%.2f in", currentPose.getY(DistanceUnit.INCH)));
        dashTelemetry.addData("IMU Heading (Yaw)", String.format(Locale.US, "%.1f deg", robotHeadingDeg));
        dashTelemetry.addData("Distance to Basket", String.format(Locale.US, "%.2f in", aim.horizontalDistanceInches));
        dashTelemetry.addData("Turret Servo Pos (Auto)", String.format(Locale.US, "%.3f", aim.servoPosition));
        dashTelemetry.addData("Continuous Motor Power", String.format(Locale.US, "%.1f%%", aim.motorPower * 100.0));
        dashTelemetry.addLine("========================================");
        dashTelemetry.update();
    }
}
