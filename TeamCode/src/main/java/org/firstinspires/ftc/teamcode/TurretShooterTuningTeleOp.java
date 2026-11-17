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
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

/**
 * TurretShooterTuningTeleOp
 *
 * Dedicated Tuning TeleOp OpMode for:
 * 1. Flywheel shooting motor power and efficiency live tuning via FTC Dashboard.
 * 2. Two-Point Servo Calibration (aim at Mark A then Mark B to compute exact servoTotalRangeDegrees).
 */
@Config
@TeleOp(name = "Turret Shooting Power Tuning (FTC Dashboard)", group = "Tuning")
public class TurretShooterTuningTeleOp extends OpMode {

    // REV HUB IMU MOUNTING CONFIGURATION
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;

    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // FTC DASHBOARD LIVE TUNING VARIABLES
    public static double POWER_MULTIPLIER = 1.0;       // Scale motor power
    public static double FLYWHEEL_EFFICIENCY = 0.65;   // Energy transfer efficiency
    public static double BASKET_X = 48.0;              // Basket X coordinate
    public static double BASKET_Y = 48.0;              // Basket Y coordinate
    public static double BASKET_HEIGHT = 43.0;         // Basket height in inches
    public static double LAUNCH_ANGLE_DEG = 50.0;      // Launcher ramp angle in degrees
    public static double FLYWHEEL_RADIUS_IN = 2.0;     // Flywheel wheel radius in inches
    public static double SERVO_TOTAL_RANGE_DEGREES = 180.0; // Single parameter name: servoTotalRangeDegrees
    public static boolean CONTINUOUS_FLYWHEEL = true;  // Continuous flywheel rotation at target RPM

    // HARDWARE DEVICES
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

    // Two-Point Servo Calibration variables
    public static double MARK_A_ANGLE_DEG = 0.0;
    public static double MARK_B_ANGLE_DEG = 90.0;
    private double posA = Double.NaN;
    private double posB = Double.NaN;
    private double currentManualServoPos = 0.5;

    private static final String SERVO_CONFIG_FILE = "turret_servo_config.txt";

    @Override
    public void init() {
        dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        try {
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(LOGO_FACING_DIR, USB_FACING_DIR);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
        } catch (Exception e) {
            imu = null;
        }

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setOffsets(TurretFieldConfig.PINPOINT_X_OFFSET_MM, TurretFieldConfig.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.resetPosAndIMU();
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        } catch (Exception e) {
            pinpoint = null;
        }

        try {
            turretServo = hardwareMap.get(Servo.class, "turretServo");
            turretServo.setPosition(aimCalculator.getForwardServoPosition());
            currentManualServoPos = aimCalculator.getForwardServoPosition();
        } catch (Exception e) {
            turretServo = null;
        }

        try {
            flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            flywheelMotor = null;
        }

        try {
            feedServo = hardwareMap.get(Servo.class, "feedServo");
            feedServo.setPosition(TurretSubsystem.FEED_REST_POSITION);
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
        dashTelemetry.addLine("Two-Point Servo Calibration: Press [A] to record Mark A, [Y] for Mark B.");
        dashTelemetry.update();
    }

    @Override
    public void loop() {
        aimCalculator.basketXInches = BASKET_X;
        aimCalculator.basketYInches = BASKET_Y;
        aimCalculator.targetHeightInches = BASKET_HEIGHT;
        aimCalculator.launchAngleDegrees = LAUNCH_ANGLE_DEG;
        aimCalculator.flywheelRadiusInches = FLYWHEEL_RADIUS_IN;
        aimCalculator.flywheelEfficiency = FLYWHEEL_EFFICIENCY;
        aimCalculator.powerMultiplier = POWER_MULTIPLIER;
        aimCalculator.servoTotalRangeDegrees = SERVO_TOTAL_RANGE_DEGREES;

        if (pinpoint != null) pinpoint.update();
        Pose2D currentPose = (pinpoint != null) ? pinpoint.getPosition() : new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

        // Two-Point Servo Calibration Button Inputs
        if (gamepad1.dpad_left) {
            currentManualServoPos = Math.max(0.0, currentManualServoPos - 0.002);
            if (turretServo != null) turretServo.setPosition(currentManualServoPos);
        } else if (gamepad1.dpad_right) {
            currentManualServoPos = Math.min(1.0, currentManualServoPos + 0.002);
            if (turretServo != null) turretServo.setPosition(currentManualServoPos);
        }

        if (gamepad1.a) {
            posA = currentManualServoPos;
        }
        if (gamepad1.y) {
            posB = currentManualServoPos;
        }

        double calculatedServoTotalRangeDegrees = Double.NaN;
        double calculatedCenterOffset = Double.NaN;

        if (!Double.isNaN(posA) && !Double.isNaN(posB) && Math.abs(posB - posA) > 0.005) {
            calculatedServoTotalRangeDegrees = (MARK_B_ANGLE_DEG - MARK_A_ANGLE_DEG) / (posB - posA);
            calculatedCenterOffset = posA - (MARK_A_ANGLE_DEG / calculatedServoTotalRangeDegrees);

            if (gamepad1.b) {
                try {
                    File file = AppUtil.getInstance().getSettingsFile(SERVO_CONFIG_FILE);
                    ReadWriteFile.writeFile(file, String.format(Locale.US, "%.2f,%.4f", calculatedServoTotalRangeDegrees, calculatedCenterOffset));
                    SERVO_TOTAL_RANGE_DEGREES = calculatedServoTotalRangeDegrees;
                } catch (Exception ignored) {}
            }
        }

        // Auto-Aim Calculation
        TurretAimCalculator.AimResult aim = aimCalculator.calculateAim(currentPose);
        if (!gamepad1.dpad_left && !gamepad1.dpad_right && turretServo != null) {
            turretServo.setPosition(aim.servoPosition);
        }

        if (flywheelMotor != null && CONTINUOUS_FLYWHEEL) {
            flywheelMotor.setPower(aim.targetInReach ? aim.motorPower : 0.0);
        }

        // Telemetry Output
        dashTelemetry.addLine("========================================");
        dashTelemetry.addLine("    TWO-POINT SERVO & POWER TUNING      ");
        dashTelemetry.addLine("========================================");
        dashTelemetry.addLine("TWO-POINT SERVO CALIBRATION:");
        dashTelemetry.addData("Mark A Angle / Pos", String.format(Locale.US, "%.1f deg / %.3f", MARK_A_ANGLE_DEG, posA));
        dashTelemetry.addData("Mark B Angle / Pos", String.format(Locale.US, "%.1f deg / %.3f", MARK_B_ANGLE_DEG, posB));

        if (!Double.isNaN(calculatedServoTotalRangeDegrees)) {
            dashTelemetry.addData("--> Calculated servoTotalRangeDegrees", String.format(Locale.US, "%.2f deg", calculatedServoTotalRangeDegrees));
            dashTelemetry.addData("--> Calculated Center Position", String.format(Locale.US, "%.4f", calculatedCenterOffset));
            dashTelemetry.addLine("Press [B] to SAVE servo configuration to settings file!");
        }

        dashTelemetry.addLine("----------------------------------------");
        dashTelemetry.addData("Live POWER_MULTIPLIER", String.format(Locale.US, "%.3f", POWER_MULTIPLIER));
        dashTelemetry.addData("Live servoTotalRangeDegrees", String.format(Locale.US, "%.1f deg", SERVO_TOTAL_RANGE_DEGREES));
        dashTelemetry.addData("Lock Status", aim.statusMessage);
        dashTelemetry.addData("Robot Pose X, Y", String.format(Locale.US, "%.2f, %.2f in", currentPose.getX(DistanceUnit.INCH), currentPose.getY(DistanceUnit.INCH)));
        dashTelemetry.addData("Distance to Basket", String.format(Locale.US, "%.2f in", aim.horizontalDistanceInches));
        dashTelemetry.addLine("========================================");
        dashTelemetry.update();
    }
}
