package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * TurretSubsystem
 *
 * Full subsystem encapsulating Odometry pose tracking, Turret Servo aiming,
 * Flywheel Motor (open or closed-loop), non-blocking Feed state machine, and safety gating.
 */
public class TurretSubsystem {

    // Flywheel Hardware Specs
    // TODO(user): MEASURE ME
    public static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    public static final double FLYWHEEL_GEAR_RATIO = 1.0;

    // Flywheel Velocity PIDF Constants (for Closed-Loop mode)
    public static double FLYWHEEL_P = 10.0;
    public static double FLYWHEEL_I = 3.0;
    public static double FLYWHEEL_D = 0.0;
    public static double FLYWHEEL_F = 12.0;

    // Flywheel Speed Tolerance (RPM)
    public static double FLYWHEEL_TOLERANCE_RPM = 100.0;

    // Feed Servo Timing & Positions
    public static double FEED_REST_POSITION = 0.0;
    public static double FEED_SHOOT_POSITION = 0.5;
    public static long FEED_PUSH_DURATION_MS = 250;
    public static long FEED_RETRACT_DURATION_MS = 250;
    public static long FEED_RECOVER_TIMEOUT_MS = 500;

    // Turn-rate threshold for firing gating (degrees per second)
    public static double TURN_RATE_THRESHOLD_DEG_PER_SEC = 45.0;

    // Lead compensation time constant (seconds)
    public static double LEAD_SECONDS = 0.0;

    // SUBSYSTEM STATE & FLAGS
    public Alliance alliance = Alliance.RED;
    public boolean useClosedLoopFlywheel = false; // Default open-loop
    public boolean flywheelEnabled = true;        // Default true

    // REV Hub IMU orientation parameters (Optional diagnostic ONLY)
    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDir = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDir = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // Distance-Dependent Efficiency Table
    public final EfficiencyTable efficiencyTable = new EfficiencyTable();

    // Core Aim Calculator Engine
    public final TurretAimCalculator aimCalculator = new TurretAimCalculator();

    // Hardware Devices
    private IMU imuDiagnostic; // Optional diagnostic only
    private GoBildaPinpointDriver pinpoint; // REQUIRED
    private Servo turretServo;               // REQUIRED
    private DcMotorEx flywheelMotor;         // REQUIRED
    private Servo feedServo;                // Optional

    // Loud Failures Error Tracking
    public final List<String> initErrors = new ArrayList<>();
    private boolean pinpointHealthy = false;

    // Single Pose Cache (Single Pose Source)
    private Pose2D cachedRobotPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private double lastHeadingDeg = 0.0;
    private long lastLoopTimeMs = 0;
    private double currentTurnRateDegPerSec = 0.0;
    private double imuYawDiagnosticDeg = 0.0;

    // Flywheel State
    private double targetFlywheelRPM = 0.0;
    private double measuredFlywheelRPM = 0.0;
    private boolean flywheelReady = false;

    // Feed Gating & State Machine
    public enum FeedState { IDLE, PUSH, RETRACT, RECOVER }
    private FeedState feedState = FeedState.IDLE;
    private long feedStateStartTimeMs = 0;

    public boolean canFire = false;
    public String blockedReason = "INITIALIZING";

    /**
     * Initializes hardware devices, checks health, configures pinpoint odometry,
     * and sets turret servo facing DIRECTLY FORWARD.
     */
    public void init(HardwareMap hardwareMap) {
        initErrors.clear();
        aimCalculator.resetTurretAngleMemory();

        // Load overrides from saved settings files if present
        loadSavedSettingsOverrides();

        // Set Basket position from current selected StartProfile
        TurretFieldConfig.StartProfile profile = TurretFieldConfig.getSelectedProfile();
        aimCalculator.basketXInches = profile.basketDxInches;
        aimCalculator.basketYInches = profile.basketDyInches;

        // Load Efficiency Table from CSV file
        efficiencyTable.loadFromFile();

        // 1. Initialize REQUIRED goBILDA Pinpoint Driver
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setOffsets(TurretFieldConfig.PINPOINT_X_OFFSET_MM, TurretFieldConfig.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);

            if (TurretFieldConfig.USE_CUSTOM_INCHES_PER_TICK) {
                double ticksPerInch = 1.0 / TurretFieldConfig.INCHES_PER_TICK;
                pinpoint.setEncoderResolution(ticksPerInch, DistanceUnit.INCH);
            } else {
                pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            }

            pinpoint.setEncoderDirections(TurretFieldConfig.PINPOINT_X_DIRECTION, TurretFieldConfig.PINPOINT_Y_DIRECTION);
            pinpoint.resetPosAndIMU();
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, profile.startHeadingDeg));
            pinpointHealthy = true;
        } catch (Exception e) {
            pinpoint = null;
            pinpointHealthy = false;
            initErrors.add("REQUIRED hardware missing: 'pinpoint' (GoBildaPinpointDriver)");
        }

        // 2. Initialize REQUIRED Turret Servo
        try {
            turretServo = hardwareMap.get(Servo.class, "turretServo");
            turretServo.setPosition(aimCalculator.getForwardServoPosition());
        } catch (Exception e) {
            turretServo = null;
            initErrors.add("REQUIRED hardware missing: 'turretServo' (Servo)");
        }

        // 3. Initialize REQUIRED Flywheel Motor
        try {
            flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
            if (useClosedLoopFlywheel) {
                flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F));
            } else {
                flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            flywheelMotor = null;
            initErrors.add("REQUIRED hardware missing: 'flywheelMotor' (DcMotorEx)");
        }

        // 4. Initialize OPTIONAL Feed Servo
        try {
            feedServo = hardwareMap.get(Servo.class, "feedServo");
            feedServo.setPosition(FEED_REST_POSITION);
        } catch (Exception e) {
            feedServo = null;
        }

        // 5. Initialize OPTIONAL REV IMU (Diagnostic Only)
        try {
            imuDiagnostic = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDir, usbFacingDir);
            imuDiagnostic.initialize(new IMU.Parameters(orientationOnRobot));
        } catch (Exception e) {
            imuDiagnostic = null;
        }

        lastLoopTimeMs = System.currentTimeMillis();
    }

    /** Display telemetry during init_loop showing profile selection and UNCONFIRMED CALIBRATIONS */
    public void displayInitLoopTelemetry(Telemetry telemetry) {
        TurretFieldConfig.StartProfile profile = TurretFieldConfig.getSelectedProfile();

        telemetry.addLine("=== INIT LOOP: SELECT START PROFILE ===");
        telemetry.addLine("Use D-PAD Left/Right to change Start Profile.");
        telemetry.addData("Selected Profile", profile.name);
        telemetry.addData("Placement Description", profile.description);
        telemetry.addData("Start Heading", profile.startHeadingDeg + " deg");
        telemetry.addData("Basket Dx, Dy", String.format(Locale.US, "%.1f, %.1f in", profile.basketDxInches, profile.basketDyInches));
        telemetry.addLine("----------------------------------------");
        telemetry.addLine("REMINDER: Place robot on START MARK flush against: " + profile.description);
        telemetry.addLine("DO NOT MOVE ROBOT AFTER INIT!");

        List<String> unconfirmed = TurretFieldConfig.getUnconfirmedCalibrations();
        if (!unconfirmed.isEmpty()) {
            telemetry.addLine("----------------------------------------");
            telemetry.addLine("=== UNCONFIRMED CALIBRATION WARNING ===");
            for (String item : unconfirmed) {
                telemetry.addLine("⚠️ UNCONFIRMED: " + item);
            }
        }
        telemetry.update();
    }

    /** System health check: Returns true ONLY if all required hardware is present and pinpoint is READY */
    public boolean isHealthy() {
        return initErrors.isEmpty() && pinpointHealthy;
    }

    /** Sets robot start pose on field */
    public void setStartPose(double xIn, double yIn, double headingDeg) {
        if (pinpoint != null) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.DEGREES, headingDeg));
        }
        cachedRobotPose = new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.DEGREES, headingDeg);
        PoseStorage.currentPose = cachedRobotPose;
        PoseStorage.hasSavedPose = true;
    }

    /** Reset robot to selected start profile pose */
    public void resetToStartPose() {
        TurretFieldConfig.StartProfile profile = TurretFieldConfig.getSelectedProfile();
        setStartPose(0.0, 0.0, profile.startHeadingDeg);
        if (imuDiagnostic != null) {
            try {
                imuDiagnostic.resetYaw();
            } catch (Exception ignored) {}
        }
    }

    /** Driver re-zero pose against wall or corner */
    public void resetPose(double xIn, double yIn, double headingDeg) {
        setStartPose(xIn, yIn, headingDeg);
        if (imuDiagnostic != null) {
            try {
                imuDiagnostic.resetYaw();
            } catch (Exception ignored) {}
        }
    }

    /**
     * MAIN AUTO-AIM AND FEED FUNCTION.
     *
     * Single Pose Source: Calls pinpoint.update() EXACTLY ONCE per loop frame.
     */
    public TurretAimCalculator.AimResult updateAutoAim(boolean triggerPressed) {
        long now = System.currentTimeMillis();
        double dt = (now - lastLoopTimeMs) / 1000.0;
        if (dt <= 0) dt = 0.001;
        lastLoopTimeMs = now;

        // Check Pinpoint Health
        if (pinpoint != null) {
            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            pinpointHealthy = (status == GoBildaPinpointDriver.DeviceStatus.READY);

            if (pinpointHealthy) {
                // Call pinpoint.update() EXACTLY ONCE
                pinpoint.update();
                cachedRobotPose = pinpoint.getPosition();
                PoseStorage.currentPose = cachedRobotPose;
                PoseStorage.hasSavedPose = true;
            }
        } else {
            pinpointHealthy = false;
        }

        // Calculate turn rate (degrees per second)
        double headingDeg = cachedRobotPose.getHeading(AngleUnit.DEGREES);
        currentTurnRateDegPerSec = TurretAimCalculator.normalizeAngleDegrees(headingDeg - lastHeadingDeg) / dt;
        lastHeadingDeg = headingDeg;

        // Diagnostic IMU yaw check (NEVER used for aiming)
        if (imuDiagnostic != null) {
            try {
                imuYawDiagnosticDeg = imuDiagnostic.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            } catch (Exception ignored) {}
        }

        // Apply distance-dependent efficiency from table
        double distanceToBasket = Math.hypot(aimCalculator.basketXInches - cachedRobotPose.getX(DistanceUnit.INCH),
                                             aimCalculator.basketYInches - cachedRobotPose.getY(DistanceUnit.INCH));
        aimCalculator.flywheelEfficiency = efficiencyTable.effectiveEfficiency(distanceToBasket);

        // Turn-rate Lead Compensation
        Pose2D leadPose = cachedRobotPose;
        if (LEAD_SECONDS > 0) {
            double leadHeading = headingDeg + currentTurnRateDegPerSec * LEAD_SECONDS;
            leadPose = new Pose2D(DistanceUnit.INCH, cachedRobotPose.getX(DistanceUnit.INCH), cachedRobotPose.getY(DistanceUnit.INCH), AngleUnit.DEGREES, leadHeading);
        }

        // Compute Aim Solution
        TurretAimCalculator.AimResult aim = aimCalculator.calculateAim(leadPose);

        // LOUD FAILURES: If system is unhealthy, DO NOT move turret or spin flywheel!
        if (!isHealthy()) {
            if (flywheelMotor != null) flywheelMotor.setPower(0);
            canFire = false;
            if (!initErrors.isEmpty()) {
                blockedReason = "HARDWARE MISSING: " + initErrors.get(0);
            } else {
                blockedReason = "PINPOINT SENSOR NOT READY";
            }
            aim.statusMessage = blockedReason;
            aim.shotValid = false;
            return aim;
        }

        // Apply Turret Servo Position
        if (turretServo != null && !aim.turretAtLimit) {
            turretServo.setPosition(aim.servoPosition);
        }

        // Flywheel Velocity Control
        targetFlywheelRPM = aim.requiredFlywheelRPM;
        if (flywheelMotor != null) {
            try {
                double ticksPerSec = flywheelMotor.getVelocity();
                measuredFlywheelRPM = (ticksPerSec / (FLYWHEEL_TICKS_PER_REV * FLYWHEEL_GEAR_RATIO)) * 60.0;
            } catch (Exception e) {
                measuredFlywheelRPM = 0.0;
            }

            if (!flywheelEnabled || !aim.targetInReach) {
                flywheelMotor.setPower(0.0);
                flywheelReady = false;
            } else if (useClosedLoopFlywheel) {
                double targetTicksPerSec = (targetFlywheelRPM / 60.0) * FLYWHEEL_TICKS_PER_REV * FLYWHEEL_GEAR_RATIO;
                flywheelMotor.setVelocity(targetTicksPerSec);
                flywheelReady = Math.abs(measuredFlywheelRPM - targetFlywheelRPM) <= FLYWHEEL_TOLERANCE_RPM;
            } else {
                flywheelMotor.setPower(aim.motorPower);
                flywheelReady = (measuredFlywheelRPM > 0) ? Math.abs(measuredFlywheelRPM - targetFlywheelRPM) <= FLYWHEEL_TOLERANCE_RPM : true;
            }
        }

        // Fire Gating Evaluation
        boolean turnRateOk = Math.abs(currentTurnRateDegPerSec) < TURN_RATE_THRESHOLD_DEG_PER_SEC;
        boolean flywheelOk = useClosedLoopFlywheel ? flywheelReady : true;

        canFire = aim.shotValid && !aim.turretAtLimit && flywheelOk && turnRateOk && pinpointHealthy;

        if (!pinpointHealthy) {
            blockedReason = "PINPOINT UNHEALTHY";
        } else if (aim.turretAtLimit) {
            blockedReason = "TURRET AT LIMIT";
        } else if (!aim.targetInReach) {
            blockedReason = "NO BALLISTIC SOLUTION";
        } else if (!aim.descending) {
            blockedReason = "ASCENDING SHOT INVALID";
        } else if (aim.powerSaturated) {
            blockedReason = "MOTOR POWER SATURATED";
        } else if (useClosedLoopFlywheel && !flywheelReady) {
            blockedReason = "FLYWHEEL NOT READY";
        } else if (!turnRateOk) {
            blockedReason = "ROBOT TURNING TOO FAST";
        } else {
            blockedReason = "READY TO FIRE";
        }

        // Feed State Machine (Non-blocking: IDLE -> PUSH -> RETRACT -> RECOVER)
        updateFeedStateMachine(now, triggerPressed);

        return aim;
    }

    private void updateFeedStateMachine(long nowMs, boolean triggerPressed) {
        if (feedServo == null) return;

        switch (feedState) {
            case IDLE:
                feedServo.setPosition(FEED_REST_POSITION);
                if (triggerPressed && canFire) {
                    feedState = FeedState.PUSH;
                    feedStateStartTimeMs = nowMs;
                    feedServo.setPosition(FEED_SHOOT_POSITION);
                }
                break;

            case PUSH:
                feedServo.setPosition(FEED_SHOOT_POSITION);
                if (nowMs - feedStateStartTimeMs >= FEED_PUSH_DURATION_MS) {
                    feedState = FeedState.RETRACT;
                    feedStateStartTimeMs = nowMs;
                    feedServo.setPosition(FEED_REST_POSITION);
                }
                break;

            case RETRACT:
                feedServo.setPosition(FEED_REST_POSITION);
                if (nowMs - feedStateStartTimeMs >= FEED_RETRACT_DURATION_MS) {
                    feedState = FeedState.RECOVER;
                    feedStateStartTimeMs = nowMs;
                }
                break;

            case RECOVER:
                feedServo.setPosition(FEED_REST_POSITION);
                boolean recovered = useClosedLoopFlywheel ? flywheelReady : true;
                boolean timeout = (nowMs - feedStateStartTimeMs >= FEED_RECOVER_TIMEOUT_MS);
                if (recovered || timeout) {
                    feedState = FeedState.IDLE;
                }
                break;
        }
    }

    public TurretAimCalculator.AimResult updateAutoAim() {
        return updateAutoAim(false);
    }

    public Pose2D getCurrentPose() {
        return cachedRobotPose;
    }

    private void loadSavedSettingsOverrides() {
        try {
            File iptFile = AppUtil.getInstance().getSettingsFile(InchesPerTickCalibrationOpMode.FILE_NAME);
            if (iptFile.exists()) {
                String str = ReadWriteFile.readFile(iptFile).trim();
                TurretFieldConfig.INCHES_PER_TICK = Double.parseDouble(str);
                TurretFieldConfig.USE_CUSTOM_INCHES_PER_TICK = true;
                TurretFieldConfig.INCHES_PER_TICK_CONFIRMED = true;
            }
        } catch (Exception ignored) {}

        try {
            File bFile = AppUtil.getInstance().getSettingsFile(BasketAimCheckOpMode.FILE_NAME);
            if (bFile.exists()) {
                String str = ReadWriteFile.readFile(bFile).trim();
                String[] parts = str.split(",");
                if (parts.length >= 2) {
                    aimCalculator.basketXInches = Double.parseDouble(parts[0].trim());
                    aimCalculator.basketYInches = Double.parseDouble(parts[1].trim());
                }
            }
        } catch (Exception ignored) {}

        try {
            File sFile = AppUtil.getInstance().getSettingsFile("turret_servo_config.txt");
            if (sFile.exists()) {
                String str = ReadWriteFile.readFile(sFile).trim();
                String[] parts = str.split(",");
                if (parts.length >= 2) {
                    aimCalculator.servoTotalRangeDegrees = Double.parseDouble(parts[0].trim());
                    aimCalculator.servoCenterPosition = Double.parseDouble(parts[1].trim());
                    TurretFieldConfig.SERVO_RANGE_CONFIRMED = true;
                }
            }
        } catch (Exception ignored) {}
    }

    public double getMeasuredFlywheelRPM() { return measuredFlywheelRPM; }
    public double getTargetFlywheelRPM() { return targetFlywheelRPM; }
    public double getCurrentTurnRateDegPerSec() { return currentTurnRateDegPerSec; }
    public double getImuYawDiagnosticDeg() { return imuYawDiagnosticDeg; }
    public double getImuYawDeltaDeg() { return TurretAimCalculator.normalizeAngleDegrees(imuYawDiagnosticDeg - cachedRobotPose.getHeading(AngleUnit.DEGREES)); }
    public FeedState getFeedState() { return feedState; }
}
