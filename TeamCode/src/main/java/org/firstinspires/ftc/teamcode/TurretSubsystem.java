package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * TurretSubsystem
 *
 * Encapsulates all turret auto-aiming, IMU orientation tracking, goBILDA Pinpoint odometry,
 * and continuous flywheel motor power control into clean, reusable function calls.
 */
public class TurretSubsystem {

    // =========================================================================
    // CONFIGURATION VARIABLES (Set these before calling init())
    // =========================================================================

    // REV Hub IMU mounting orientation
    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDir = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDir = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // Target Basket Location (Inches)
    public double basketXInches = 48.0;
    public double basketYInches = 48.0;
    public double basketHeightInches = 43.0;

    // Hardware Specs & Launch Parameters
    public double flywheelRadiusInches = 2.0;
    public double launchAngleDegrees = 50.0;
    public double flywheelEfficiency = 0.65;
    public double powerMultiplier = 1.0;
    public double maxMotorRPM = 6000.0;

    // Turret Offsets on Robot & Servo Specs
    public double turretOffsetXInches = 0.0;
    public double turretOffsetYInches = 0.0;
    public double turretHeightInches = 12.0;
    public double servoCenterPosition = 0.5; // Position corresponding to facing DIRECTLY FORWARD
    public double servoTotalRangeDegrees = 180.0;
    public boolean reverseServoDirection = false;

    // =========================================================================
    // HARDWARE DEVICES
    // =========================================================================
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;
    private Servo turretServo;
    private DcMotorEx flywheelMotor;
    private Servo feedServo;

    // Core Aim Calculator Engine
    private final TurretAimCalculator aimCalculator = new TurretAimCalculator();

    /**
     * Initializes hardware devices, configures IMU orientation, resets odometry pose,
     * and homes the turret servo to face DIRECTLY FORWARD relative to the robot.
     *
     * @param hardwareMap HardwareMap passed from the OpMode
     */
    public void init(HardwareMap hardwareMap) {
        syncCalculatorParameters();

        // 1. Initialize REV Hub IMU
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDir, usbFacingDir);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw();
        } catch (Exception e) {
            imu = null;
        }

        // 2. Initialize goBILDA Pinpoint Odometry Computer
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                          GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.resetPosAndIMU();
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        } catch (Exception e) {
            pinpoint = null;
        }

        // 3. Initialize Turret Servo - Set to face DIRECTLY FORWARD relative to robot
        try {
            turretServo = hardwareMap.get(Servo.class, "turretServo");
            turretServo.setPosition(aimCalculator.getForwardServoPosition());
        } catch (Exception e) {
            turretServo = null;
        }

        // 4. Initialize Flywheel Motor
        try {
            flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            flywheelMotor = null;
        }

        // 5. Initialize Feed Servo (optional trigger servo)
        try {
            feedServo = hardwareMap.get(Servo.class, "feedServo");
            feedServo.setPosition(0.0);
        } catch (Exception e) {
            feedServo = null;
        }
    }

    /**
     * MAIN AUTO-AIM FUNCTION.
     *
     * Call this function repeatedly in your loop() or runOpMode()!
     * It automatically:
     *  1. Updates Odometry (X, Y) and IMU rotation angle (Heading).
     *  2. Calculates the 3D ballistic trajectory and target flywheel RPM.
     *  3. Automatically turns the Turret Servo to lock onto the basket.
     *  4. Maintains CONTINUOUS flywheel motor rotation at target calculated power.
     *  5. Actuates feed servo if trigger is pressed.
     *
     * @param feedTriggerPressed Pass true to push sample into flywheel to shoot
     * @return AimResult object containing target RPM, distance, servo position, and status
     */
    public TurretAimCalculator.AimResult updateAutoAim(boolean feedTriggerPressed) {
        syncCalculatorParameters();

        Pose2D currentPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

        if (pinpoint != null) {
            pinpoint.update();
            currentPose = pinpoint.getPosition();
        }

        double headingDeg = currentPose.getHeading(AngleUnit.DEGREES);
        if (imu != null) {
            headingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        Pose2D poseWithIMU = new Pose2D(
                DistanceUnit.INCH,
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                AngleUnit.DEGREES,
                headingDeg
        );

        // Compute 3D Ballistic Solution & Turret Servo Angle
        TurretAimCalculator.AimResult aim = aimCalculator.calculateAim(poseWithIMU);

        // Apply Turret Servo Position
        if (turretServo != null) {
            turretServo.setPosition(aim.servoPosition);
        }

        // Continuous Flywheel Motor Rotation
        if (flywheelMotor != null) {
            if (aim.targetInReach) {
                flywheelMotor.setPower(aim.motorPower);
            } else {
                flywheelMotor.setPower(0.0);
            }
        }

        // Feed Servo Trigger
        if (feedServo != null) {
            if (feedTriggerPressed && aim.targetInReach) {
                feedServo.setPosition(0.5); // Push sample into spinning wheel
            } else {
                feedServo.setPosition(0.0);
            }
        }

        return aim;
    }

    /**
     * Overloaded helper method for updateAutoAim() without feed trigger actuation.
     */
    public TurretAimCalculator.AimResult updateAutoAim() {
        return updateAutoAim(false);
    }

    /**
     * Resets odometry starting position to (0, 0, 0) and zeroes the IMU yaw.
     */
    public void resetOdometryAndIMU() {
        if (pinpoint != null) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }
        if (imu != null) {
            imu.resetYaw();
        }
    }

    /** Gets current robot Pose2D */
    public Pose2D getCurrentPose() {
        if (pinpoint != null) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double heading = (imu != null) ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) : pose.getHeading(AngleUnit.DEGREES);
            return new Pose2D(DistanceUnit.INCH, pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), AngleUnit.DEGREES, heading);
        }
        return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    }

    private void syncCalculatorParameters() {
        aimCalculator.basketXInches = basketXInches;
        aimCalculator.basketYInches = basketYInches;
        aimCalculator.basketHeightInches = basketHeightInches;
        aimCalculator.launchAngleDegrees = launchAngleDegrees;
        aimCalculator.flywheelRadiusInches = flywheelRadiusInches;
        aimCalculator.flywheelEfficiency = flywheelEfficiency;
        aimCalculator.powerMultiplier = powerMultiplier;
        aimCalculator.maxMotorRPM = maxMotorRPM;
        aimCalculator.turretOffsetXInches = turretOffsetXInches;
        aimCalculator.turretOffsetYInches = turretOffsetYInches;
        aimCalculator.turretHeightInches = turretHeightInches;
        aimCalculator.servoCenterPosition = servoCenterPosition;
        aimCalculator.servoTotalRangeDegrees = servoTotalRangeDegrees;
        aimCalculator.reverseServoDirection = reverseServoDirection;
    }
}
