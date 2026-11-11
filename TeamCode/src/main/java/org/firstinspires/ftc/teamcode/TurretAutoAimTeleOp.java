package org.firstinspires.ftc.teamcode;

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
 * TurretAutoAimTeleOp
 *
 * MAIN STARTUP FILE for auto-aiming turret and continuous flywheel shooter using IMU and goBILDA Pinpoint Odometry.
 *
 * COMPETITION BEHAVIOR:
 * 1. IMU Integration: Uses REV Hub IMU with user-configurable Logo and USB facing directions
 *    to track real-time robot rotation angle on the field.
 * 2. Continuous Flywheel Rotation: Flywheel motor ALWAYS spins continuously at the target calculated RPM
 *    so there is zero latency when firing a sample into the basket.
 * 3. Starting Turret Direction: Turret servo initializes facing DIRECTLY FORWARD relative to the robot.
 */
@TeleOp(name = "Turret Auto Aim Startup", group = "TeleOp")
public class TurretAutoAimTeleOp extends OpMode {

    // =========================================================================
    // REV HUB IMU MOUNTING CONFIGURATION (MODIFY FOR YOUR ROBOT HARDWARE SETUP)
    // =========================================================================
    // Specify direction the printed REV logo on the Hub is pointing relative to robot:
    // (UP, DOWN, FORWARD, BACKWARD, LEFT, RIGHT)
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;

    // Specify direction the USB connector on the Hub is pointing relative to robot:
    // (UP, DOWN, FORWARD, BACKWARD, LEFT, RIGHT)
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

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

    // Aiming Subsystem Calculator
    private final TurretAimCalculator aimCalculator = new TurretAimCalculator();

    // Controls & State
    private boolean autoAimEnabled = true;
    private boolean lastXButtonState = false;

    // Feed servo positions
    private static final double FEED_REST_POSITION = 0.0;
    private static final double FEED_SHOOT_POSITION = 0.5;

    @Override
    public void init() {
        // ---------------------------------------------------------------------
        // 1. INPUT CONFIGURATION VARIABLES
        // ---------------------------------------------------------------------
        aimCalculator.flywheelRadiusInches = 2.0;

        aimCalculator.turretOffsetXInches = 0.0;
        aimCalculator.turretOffsetYInches = 0.0;
        aimCalculator.turretHeightInches = 12.0;

        // Basket Position (From Step 1 calibration)
        aimCalculator.basketXInches = 48.0;
        aimCalculator.basketYInches = 48.0;
        aimCalculator.basketHeightInches = 43.0;

        aimCalculator.launchAngleDegrees = 50.0;
        aimCalculator.flywheelEfficiency = 0.65;
        aimCalculator.powerMultiplier = 1.0; // Tuned value from Step 3 tuning
        aimCalculator.maxMotorRPM = 6000.0;

        aimCalculator.servoCenterPosition = 0.5; // Facing DIRECTLY FORWARD relative to robot
        aimCalculator.servoTotalRangeDegrees = 180.0;
        aimCalculator.reverseServoDirection = false;

        // ---------------------------------------------------------------------
        // 2. HARDWARE INITIALIZATION & IMU SETUP
        // ---------------------------------------------------------------------

        // Initialize REV Hub IMU with logo and USB mounting directions
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

        // Initialize Turret Servo - Set facing DIRECTLY FORWARD relative to robot
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        turretServo.setPosition(aimCalculator.getForwardServoPosition());

        // Initialize Flywheel Motor
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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

        telemetry.addLine("=== TURRET AUTO-AIM STARTUP READY ===");
        telemetry.addData("IMU Status", imu != null ? "INITIALIZED (Logo: " + LOGO_FACING_DIR + ", USB: " + USB_FACING_DIR + ")" : "NOT FOUND (Using Odometry Heading)");
        telemetry.addLine("Turret direction: Facing DIRECTLY FORWARD (0 deg)");
        telemetry.addData("Shooter Mode", "CONTINUOUS FLYWHEEL ROTATION ACTIVE");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ---------------------------------------------------------------------
        // 1. UPDATE ODOMETRY & IMU ROTATION ANGLE
        // ---------------------------------------------------------------------
        pinpoint.update();
        Pose2D currentRobotPose = pinpoint.getPosition();

        // Retrieve real-time rotation angle from IMU (or Pinpoint fallback)
        double robotHeadingDeg;
        if (imu != null) {
            robotHeadingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        } else {
            robotHeadingDeg = currentRobotPose.getHeading(AngleUnit.DEGREES);
        }

        // Combine 2D position (X, Y) from Odometry with real-time rotation angle (Heading) from IMU
        Pose2D robotPoseWithIMU = new Pose2D(
                DistanceUnit.INCH,
                currentRobotPose.getX(DistanceUnit.INCH),
                currentRobotPose.getY(DistanceUnit.INCH),
                AngleUnit.DEGREES,
                robotHeadingDeg
        );

        // ---------------------------------------------------------------------
        // 2. GAMEPAD BUTTON CONTROLS
        // ---------------------------------------------------------------------
        boolean currentXState = gamepad1.x;
        if (currentXState && !lastXButtonState) {
            autoAimEnabled = !autoAimEnabled;
        }
        lastXButtonState = currentXState;

        if (gamepad1.a) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            if (imu != null) {
                imu.resetYaw();
            }
        }

        // ---------------------------------------------------------------------
        // 3. AUTO-AIM CALCULATIONS & CONTINUOUS SHOOTER ROTATION
        // ---------------------------------------------------------------------
        TurretAimCalculator.AimResult aim = aimCalculator.calculateAim(robotPoseWithIMU);

        if (autoAimEnabled) {
            // Apply calculated Turret Servo angle (uses real-time IMU robot heading)
            turretServo.setPosition(aim.servoPosition);

            // COMPETITION REQUIREMENT: Flywheel motor ALWAYS rotates continuously at target calculated RPM
            if (aim.targetInReach) {
                flywheelMotor.setPower(aim.motorPower);
            } else {
                flywheelMotor.setPower(0.0);
            }
        } else {
            // Manual Override Mode: Turret points directly forward
            turretServo.setPosition(aimCalculator.getForwardServoPosition());
            flywheelMotor.setPower(gamepad1.right_trigger);
        }

        // Feed Trigger Servo (pushes sample into continuously rotating flywheel)
        if (feedServo != null) {
            if (gamepad1.right_trigger > 0.5 || gamepad1.b) {
                feedServo.setPosition(FEED_SHOOT_POSITION);
            } else {
                feedServo.setPosition(FEED_REST_POSITION);
            }
        }

        // ---------------------------------------------------------------------
        // 4. MANUAL ROBOT DRIVING (IF DRIVE MOTORS PRESENT)
        // ---------------------------------------------------------------------
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

        // ---------------------------------------------------------------------
        // 5. TELEMETRY DISPLAY
        // ---------------------------------------------------------------------
        telemetry.addLine("========================================");
        telemetry.addLine("    TURRET AUTO-AIM SYSTEM (COMPETITION)");
        telemetry.addLine("========================================");
        telemetry.addData("Shooter Mode", "CONTINUOUS ROTATION ACTIVE");
        telemetry.addData("Auto-Aim Status", autoAimEnabled ? "AUTO-AIMING (IMU ON)" : "MANUAL (FORWARD)");
        telemetry.addData("Lock Status", aim.statusMessage);
        telemetry.addLine("----------------------------------------");

        telemetry.addData("Robot Pose X", String.format(Locale.US, "%.2f in", currentRobotPose.getX(DistanceUnit.INCH)));
        telemetry.addData("Robot Pose Y", String.format(Locale.US, "%.2f in", currentRobotPose.getY(DistanceUnit.INCH)));
        telemetry.addData("IMU Heading (Yaw)", String.format(Locale.US, "%.1f deg", robotHeadingDeg));
        telemetry.addData("Distance to Basket", String.format(Locale.US, "%.2f in", aim.horizontalDistanceInches));
        telemetry.addLine("----------------------------------------");

        telemetry.addData("Turret Servo Pos", String.format(Locale.US, "%.3f", aim.servoPosition));
        telemetry.addData("Target Flywheel RPM", String.format(Locale.US, "%.0f RPM", aim.requiredFlywheelRPM));
        telemetry.addData("Continuous Motor Power", String.format(Locale.US, "%.1f%%", aim.motorPower * 100.0));
        telemetry.addLine("========================================");
        telemetry.update();
    }
}
