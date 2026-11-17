package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * AutoTurretShooter
 *
 * Production-ready, modular auto-aiming rotating turret and variable-speed cylindrical shooter
 * subsystem for FTC "BioBuzz" season featuring Shoot-on-the-Move (SOTM) compensation,
 * latency lookahead, rigid-body kinematic offsets, physics & LUT ballistics, Limelight 3A hook,
 * FTC Dashboard live tuning & field overlay canvas drawing.
 *
 * ALL UNITS ARE STANDARDIZED TO INCHES (DistanceUnit.INCH) AND DEGREES/RADIANS.
 */
@Config
public class AutoTurretShooter {

    // =========================================================================
    // FIELD, ROBOT & ODOMETRY PRE-CONFIGURED CONSTANTS (INCHES)
    // =========================================================================

    // Field Dimensions: Bottom-Left (0.0, 0.0), Top-Right (144.0, 144.0) inches
    public static double FIELD_SIZE_INCHES = 144.0;

    // Initial Robot Pose (Measured at robot center: 138.0, 30.2, Facing Left 180.0 deg)
    public static double INIT_ROBOT_X = 138.0;
    public static double INIT_ROBOT_Y = 30.2;
    public static double INIT_ROBOT_HEADING_DEG = 180.0;

    // Measured Odometry Resolution: 40 inches = 23,919 ticks (~0.0016723 in/tick / 597.975 ticks/in)
    public static double ODOMETRY_INCHES_PER_TICK = 40.0 / 23919.0;

    // Pinpoint Odometry Pod Offsets (Inches)
    // Robot Center at (6.5, 6.0). X-Pod at (5.0, 2.5) -> Forward -1.5, Left -3.5. Y-Pod at (8.0, 2.5) -> Forward +1.5, Left -3.5.
    // Pinpoint driver expects: X_POD_OFFSET_IN = +1.5 (forward pod offset), Y_POD_OFFSET_IN = -3.5 (lateral pod offset)
    public static double X_POD_OFFSET_IN = 1.5;
    public static double Y_POD_OFFSET_IN = -3.5;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_X_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // Turret Mechanical Calibration Constants (Measured empirically via TurretCalibrationOpMode)
    public static double TURRET_DEGREES_PER_TICK = 0.125;   // Degrees rotated per encoder tick (from TurretCalibrationOpMode)
    public static boolean TURRET_ENCODER_REVERSED = false;   // Polarity flag: true if clockwise turn gives positive ticks
    public static double TURRET_GEAR_RATIO = 5.0;            // Motor rotations per 1 full turret rotation (informational)
    public static double TURRET_TICKS_PER_REV = 28.0;        // Motor encoder resolution (informational)
    public static double MIN_TURRET_ANGLE_DEG = -135.0;      // Soft limit minimum (relative to robot front)
    public static double MAX_TURRET_ANGLE_DEG = 135.0;       // Soft limit maximum (relative to robot front)

    public static double TURRET_OFFSET_FORWARD_IN = 0.0;     // Turret pivot Forward offset from robot center (inches)
    public static double TURRET_OFFSET_LEFT_IN = 0.0;        // Turret pivot Left offset from robot center (inches)

    public static double ROLLER_RADIUS_IN = 2.0;             // Shooter cylindrical roller radius (inches)
    public static double SHOOTER_GEAR_RATIO = 1.0;           // Motor rotations per roller rotation
    public static double SHOOTER_TICKS_PER_REV = 28.0;       // Shooter motor encoder resolution
    public static double LAUNCH_ANGLE_DEG = 50.0;            // Shooter hood launch angle (degrees)
    public static double SHOOTER_RELEASE_HEIGHT_IN = 12.0;   // Release height of projectile center above floor (inches)
    public static double EXIT_FORWARD_OFFSET_IN = 0.0;       // Distance exit point is ahead of turret pivot (inches)

    // Ballistic Model Toggles & Slip Factors
    public static boolean USE_PHYSICS_MODEL = true;          // true = physics formula, false = LUT
    public static double SMALL_SAMPLE_SLIP_FACTOR = 0.65;    // Energy transfer efficiency for SMALL samples
    public static double LARGE_SAMPLE_SLIP_FACTOR = 0.75;    // Energy transfer efficiency for LARGE samples
    public static double SMALL_SAMPLE_TOF_SCALAR = 1.05;     // Aerodynamic drag time-of-flight scalar (SMALL)
    public static double LARGE_SAMPLE_TOF_SCALAR = 1.10;     // Aerodynamic drag time-of-flight scalar (LARGE)
    public static double MANUAL_RPM_OFFSET = 0.0;            // Dashboard manual RPM offset trim

    // SOTM & Velocity Low-Pass Filter
    public static boolean ENABLE_SHOOT_ON_THE_MOVE = true;    // Global toggle for Shoot-on-the-Move
    public static double LATENCY_COMPENSATION_SEC = 0.08;   // System actuation & pipeline delay (seconds)
    public static double VELOCITY_FILTER_ALPHA = 0.20;       // EMA filter alpha (0.0 to 1.0)

    // Shooter Motor PIDF Constants
    public static double SHOOTER_P = 12.0;
    public static double SHOOTER_I = 3.0;
    public static double SHOOTER_D = 0.0;
    public static double SHOOTER_F = 13.5;

    // Dynamic Readiness Check Tolerances & Safety Limits
    public static double TURRET_TOLERANCE_DEG = 2.5;         // Allowable turret angle error (degrees)
    public static double VELOCITY_TOLERANCE_TPS = 120.0;     // Allowable shooter motor velocity error (ticks/sec)
    public static double MIN_SHOOT_DISTANCE_IN = 18.0;       // Minimum shoot distance (inches)
    public static double MAX_SHOOT_DISTANCE_IN = 144.0;      // Maximum shoot distance (inches)
    public static double MAX_SOTM_LINEAR_SPEED = 60.0;       // Max robot linear speed allowed to fire (in/sec)
    public static double MAX_SOTM_LINEAR_SPEED_IN_PER_SEC = 60.0; // Dashboard alias
    public static double MAX_SOTM_ANGULAR_SPEED = Math.toRadians(120.0); // Max robot turn rate allowed to fire (rad/sec)

    // Basket Receiving Hemisphere Angle Check
    public static boolean ENABLE_BASKET_ENTRY_ANGLE_CHECK = true;
    public static double MAX_ENTRY_ANGLE_DEV_DEG = 75.0;     // Maximum allowable deviation from basket opening normal (degrees)

    // Feed / Indexer Servo Positions
    public static double FEED_REST_POSITION = 0.0;
    public static double FEED_SHOOT_POSITION = 0.5;

    // Gravity constant in inches/s^2 (9.81 m/s^2 * 39.3701)
    public static final double G_IN_PER_SEC2 = 386.09;

    // =========================================================================
    // ENUMS & INNER CLASSES
    // =========================================================================

    public enum AllianceColor {
        RED,
        BLUE
    }

    public enum TargetBasketMode {
        AUTO_NEAREST,
        RED_RIGHT,
        RED_LEFT,
        BLUE_RIGHT,
        BLUE_LEFT
    }

    public enum SampleType {
        SMALL,
        LARGE
    }

    public static class Basket {
        public String name;
        public double x;                    // Field X (inches)
        public double y;                    // Field Y (inches)
        public double z;                    // Field Z height (inches)
        public double openingDirectionDeg;  // Direction basket opening faces (degrees)
        public boolean isShootable;         // True if active and shootable

        public Basket(String name, double x, double y, double z, double openingDirectionDeg, boolean isShootable) {
            this.name = name;
            this.x = x;
            this.y = y;
            this.z = z;
            this.openingDirectionDeg = openingDirectionDeg;
            this.isShootable = isShootable;
        }
    }

    public static class LUTPoint {
        public double distanceInches;
        public double rpm;
        public double timeOfFlightSec;

        public LUTPoint(double distanceInches, double rpm, double timeOfFlightSec) {
            this.distanceInches = distanceInches;
            this.rpm = rpm;
            this.timeOfFlightSec = timeOfFlightSec;
        }
    }

    public static class AimResult {
        public Basket targetBasket;
        public double turretFieldX;
        public double turretFieldY;
        public double virtualBasketX;
        public double virtualBasketY;
        public double staticDistanceInches;
        public double effectiveDistanceInches;
        public double robotLinearSpeedInSec;
        public double robotAngularSpeedRadSec;
        public double targetTurretAngleDeg;
        public double actualTurretAngleDeg;
        public double turretAngleErrorDeg;
        public double targetShooterVelocityTPS;
        public double actualShooterVelocityTPS;
        public double shooterVelocityErrorTPS;
        public double targetShooterRPM;
        public double actualShooterRPM;
        public double estimatedTimeOfFlightSec;
        public boolean isTurretClamped;
        public boolean isEntryAngleValid;
        public boolean isReadyToShoot;
        public String blockedReason;
    }

    // =========================================================================
    // FIXED FIELD BASKET DEFINITIONS (BioBuzz Field Coordinates in Inches)
    // =========================================================================

    public final Basket BASKET_RED_RIGHT  = new Basket("RED_RIGHT",  84.0, 60.0, 56.0, 0.0,   true);
    public final Basket BASKET_RED_LEFT   = new Basket("RED_LEFT",   84.0, 132.0, 56.0, 0.0,  false);
    public final Basket BASKET_BLUE_RIGHT = new Basket("BLUE_RIGHT", 60.0,  12.0, 56.0, 180.0, false);
    public final Basket BASKET_BLUE_LEFT  = new Basket("BLUE_LEFT",  60.0,  84.0, 56.0, 180.0, false);

    // Lookup Tables for SMALL and LARGE samples
    private final List<LUTPoint> smallSampleLUT = new ArrayList<>();
    private final List<LUTPoint> largeSampleLUT = new ArrayList<>();

    // Hardware Devices
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private DcMotorEx turretMotor;
    private DcMotorEx shooterMotor;
    private Servo feedServo;

    // Subsystem State
    private AllianceColor alliance = AllianceColor.RED;
    private final AimResult lastAimResult = new AimResult();

    // EMA Low-Pass Filtered Velocity Memory
    private double filteredVx = 0.0;
    private double filteredVy = 0.0;
    private double filteredOmega = 0.0;
    private boolean filterInitialized = false;

    // Loud Failures
    public final List<String> initErrors = new ArrayList<>();

    public AutoTurretShooter() {
        populateDefaultLUTs();
    }

    // =========================================================================
    // INITIALIZATION
    // =========================================================================

    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, new Pose2D(DistanceUnit.INCH, INIT_ROBOT_X, INIT_ROBOT_Y, AngleUnit.DEGREES, INIT_ROBOT_HEADING_DEG), AllianceColor.RED);
    }

    public void init(HardwareMap hardwareMap, Pose2D initialPose, AllianceColor alliance) {
        this.alliance = alliance;
        initErrors.clear();
        filterInitialized = false;

        // 1. Pinpoint Odometry Driver (Set exact resolution to 597.975 ticks/inch)
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setOffsets(X_POD_OFFSET_IN, Y_POD_OFFSET_IN, DistanceUnit.INCH);
            double ticksPerInch = 1.0 / ODOMETRY_INCHES_PER_TICK; // 23919.0 / 40.0 = 597.975 ticks/in
            pinpoint.setEncoderResolution(ticksPerInch, DistanceUnit.INCH);
            pinpoint.setEncoderDirections(PINPOINT_X_DIRECTION, PINPOINT_Y_DIRECTION);
            pinpoint.resetPosAndIMU();
            if (initialPose != null) {
                pinpoint.setPosition(initialPose);
            }
        } catch (Exception e) {
            pinpoint = null;
            initErrors.add("REQUIRED hardware missing: 'pinpoint' (GoBildaPinpointDriver)");
        }

        // 2. Limelight 3A Vision Sensor (Optional Hook)
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
        } catch (Exception e) {
            limelight = null; // Optional vision hook
        }

        // 3. Turret Motor (Position PID Control)
        try {
            turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setTargetPosition(0);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMotor.setPower(0.8);
        } catch (Exception e) {
            turretMotor = null;
            initErrors.add("REQUIRED hardware missing: 'turretMotor' (DcMotorEx)");
        }

        // 4. Shooter Motor (Closed-Loop Velocity Control)
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F));
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            shooterMotor = null;
            initErrors.add("REQUIRED hardware missing: 'shooterMotor' (DcMotorEx)");
        }

        // 5. Feed Servo (Optional)
        try {
            feedServo = hardwareMap.get(Servo.class, "feedServo");
            feedServo.setPosition(FEED_REST_POSITION);
        } catch (Exception e) {
            feedServo = null;
        }
    }

    public boolean isHealthy() {
        return initErrors.isEmpty() && (pinpoint != null && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY);
    }

    public double rawTicksToInches(int ticks) {
        return ticks * ODOMETRY_INCHES_PER_TICK;
    }

    // =========================================================================
    // TURRET ANGLE <-> TICK CONVERSION HELPERS
    // =========================================================================

    public int turretDegreesToTicks(double degrees) {
        if (TURRET_DEGREES_PER_TICK <= 0) return 0;
        double rawTicks = degrees / TURRET_DEGREES_PER_TICK;
        return (int) Math.round(TURRET_ENCODER_REVERSED ? -rawTicks : rawTicks);
    }

    public double ticksToTurretDegrees(int ticks) {
        double rawDegrees = ticks * TURRET_DEGREES_PER_TICK;
        return TURRET_ENCODER_REVERSED ? -rawDegrees : rawDegrees;
    }

    // =========================================================================
    // LIMELIGHT 3A HOOK & DYNAMIC TARGETING API
    // =========================================================================

    public void setBasketShootable(TargetBasketMode mode, boolean isShootable) {
        switch (mode) {
            case RED_RIGHT:  BASKET_RED_RIGHT.isShootable = isShootable; break;
            case RED_LEFT:   BASKET_RED_LEFT.isShootable = isShootable; break;
            case BLUE_RIGHT: BASKET_BLUE_RIGHT.isShootable = isShootable; break;
            case BLUE_LEFT:  BASKET_BLUE_LEFT.isShootable = isShootable; break;
            default: break;
        }
    }

    public void updateValidBasketsFromLimelight(Limelight3A limelight) {
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                setBasketShootable(TargetBasketMode.RED_RIGHT, true);
            }
        }
    }

    // =========================================================================
    // MAIN LOOP EXECUTION
    // =========================================================================

    public AimResult updateAutoAim(SampleType currentSample, boolean spinUpShooter, boolean enableShootOnTheMove) {
        return updateAutoAim(currentSample, TargetBasketMode.AUTO_NEAREST, spinUpShooter, enableShootOnTheMove);
    }

    public AimResult updateAutoAim(SampleType currentSample, TargetBasketMode basketMode, boolean spinUpShooter, boolean enableShootOnTheMove) {
        AimResult result = lastAimResult;

        if (!isHealthy()) {
            if (shooterMotor != null) shooterMotor.setPower(0);
            result.isReadyToShoot = false;
            result.blockedReason = initErrors.isEmpty() ? "PINPOINT NOT READY" : "HARDWARE MISSING: " + initErrors.get(0);
            return result;
        }

        // 0. Update Limelight 3A Vision Hook if present
        if (limelight != null) {
            updateValidBasketsFromLimelight(limelight);
        }

        // 1. Extract Pose & Velocity from Pinpoint (INCHES & RADIANS)
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        double robotX = pose.getX(DistanceUnit.INCH);
        double robotY = pose.getY(DistanceUnit.INCH);
        double robotHeadingRad = pose.getHeading(AngleUnit.RADIANS);

        double rawVxRobot = pinpoint.getVelX(DistanceUnit.INCH);
        double rawVyRobot = pinpoint.getVelY(DistanceUnit.INCH);
        double rawOmega = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        // Convert robot-frame velocity to field-frame velocity
        double rawFieldVx = rawVxRobot * Math.cos(robotHeadingRad) - rawVyRobot * Math.sin(robotHeadingRad);
        double rawFieldVy = rawVxRobot * Math.sin(robotHeadingRad) + rawVyRobot * Math.cos(robotHeadingRad);

        // EMA Low-Pass Filter
        if (!filterInitialized) {
            filteredVx = rawFieldVx;
            filteredVy = rawFieldVy;
            filteredOmega = rawOmega;
            filterInitialized = true;
        } else {
            filteredVx = VELOCITY_FILTER_ALPHA * rawFieldVx + (1.0 - VELOCITY_FILTER_ALPHA) * filteredVx;
            filteredVy = VELOCITY_FILTER_ALPHA * rawFieldVy + (1.0 - VELOCITY_FILTER_ALPHA) * filteredVy;
            filteredOmega = VELOCITY_FILTER_ALPHA * rawOmega + (1.0 - VELOCITY_FILTER_ALPHA) * filteredOmega;
        }

        result.robotLinearSpeedInSec = Math.hypot(filteredVx, filteredVy);
        result.robotAngularSpeedRadSec = filteredOmega;

        // 2. Rigid-Body Kinematics (Turret Pivot Position & Velocity)
        double cosH = Math.cos(robotHeadingRad);
        double sinH = Math.sin(robotHeadingRad);

        double turretX = robotX + TURRET_OFFSET_FORWARD_IN * cosH - TURRET_OFFSET_LEFT_IN * sinH;
        double turretY = robotY + TURRET_OFFSET_FORWARD_IN * sinH + TURRET_OFFSET_LEFT_IN * cosH;

        double turretVx = filteredVx - filteredOmega * (TURRET_OFFSET_FORWARD_IN * sinH + TURRET_OFFSET_LEFT_IN * cosH);
        double turretVy = filteredVy + filteredOmega * (TURRET_OFFSET_FORWARD_IN * cosH - TURRET_OFFSET_LEFT_IN * sinH);

        result.turretFieldX = turretX;
        result.turretFieldY = turretY;

        // 3. Latency Lookahead
        double predHeading = robotHeadingRad + filteredOmega * LATENCY_COMPENSATION_SEC;
        double predTurretX = turretX + turretVx * LATENCY_COMPENSATION_SEC;
        double predTurretY = turretY + turretVy * LATENCY_COMPENSATION_SEC;

        // 4. Select Target Basket
        Basket basket = selectBasket(basketMode, turretX, turretY);
        result.targetBasket = basket;

        result.staticDistanceInches = Math.hypot(basket.x - turretX, basket.y - turretY);

        // 5. Iterative Virtual Target Solver (SOTM Compensation)
        double virtBasketX = basket.x;
        double virtBasketY = basket.y;
        double effectiveDist = Math.hypot(virtBasketX - predTurretX, virtBasketY - predTurretY);

        double sampleSlip = (currentSample == SampleType.SMALL) ? SMALL_SAMPLE_SLIP_FACTOR : LARGE_SAMPLE_SLIP_FACTOR;
        double tofScalar = (currentSample == SampleType.SMALL) ? SMALL_SAMPLE_TOF_SCALAR : LARGE_SAMPLE_TOF_SCALAR;

        boolean activeSOTM = ENABLE_SHOOT_ON_THE_MOVE && enableShootOnTheMove;

        if (activeSOTM) {
            for (int i = 0; i < 3; i++) {
                double calcDist = Math.max(0.1, effectiveDist - EXIT_FORWARD_OFFSET_IN);
                double estV0 = calculateLaunchVelocityV0(calcDist, basket.z, currentSample, sampleSlip);
                double launchAngleRad = Math.toRadians(LAUNCH_ANGLE_DEG);

                double estTof = (estV0 > 1.0) ? (calcDist / (estV0 * Math.cos(launchAngleRad))) * tofScalar : 0.5;

                virtBasketX = basket.x - turretVx * estTof;
                virtBasketY = basket.y - turretVy * estTof;
                effectiveDist = Math.hypot(virtBasketX - predTurretX, virtBasketY - predTurretY);
            }
        }

        result.virtualBasketX = virtBasketX;
        result.virtualBasketY = virtBasketY;
        result.effectiveDistanceInches = Math.max(0.1, effectiveDist - EXIT_FORWARD_OFFSET_IN);

        // 6. Global & Robot-Relative Turret Angle
        double globalAimAngleRad = Math.atan2(virtBasketY - predTurretY, virtBasketX - predTurretX);
        double rawTargetTurretAngleDeg = Math.toDegrees(AngleUnit.normalizeRadians(globalAimAngleRad - predHeading));

        // Soft limit clamping
        if (rawTargetTurretAngleDeg < MIN_TURRET_ANGLE_DEG) {
            result.targetTurretAngleDeg = MIN_TURRET_ANGLE_DEG;
            result.isTurretClamped = true;
        } else if (rawTargetTurretAngleDeg > MAX_TURRET_ANGLE_DEG) {
            result.targetTurretAngleDeg = MAX_TURRET_ANGLE_DEG;
            result.isTurretClamped = true;
        } else {
            result.targetTurretAngleDeg = rawTargetTurretAngleDeg;
            result.isTurretClamped = false;
        }

        // Command Turret Motor
        if (turretMotor != null) {
            int targetTicks = turretDegreesToTicks(result.targetTurretAngleDeg);
            turretMotor.setTargetPosition(targetTicks);
        }

        // Measure actual turret angle
        if (turretMotor != null) {
            int currentTicks = turretMotor.getCurrentPosition();
            result.actualTurretAngleDeg = ticksToTurretDegrees(currentTicks);
        }
        result.turretAngleErrorDeg = Math.abs(result.actualTurretAngleDeg - result.targetTurretAngleDeg);

        // 7. Calculate Shooter Motor Velocity
        double targetV0 = calculateLaunchVelocityV0(result.effectiveDistanceInches, basket.z, currentSample, sampleSlip);
        double launchRad = Math.toRadians(LAUNCH_ANGLE_DEG);
        result.estimatedTimeOfFlightSec = (targetV0 > 1.0) ? (result.effectiveDistanceInches / (targetV0 * Math.cos(launchRad))) * tofScalar : 0.0;

        if (USE_PHYSICS_MODEL) {
            double vRoller = targetV0 / sampleSlip;
            double omegaRoller = vRoller / ROLLER_RADIUS_IN;
            double omegaMotor = omegaRoller * SHOOTER_GEAR_RATIO;
            result.targetShooterVelocityTPS = (omegaMotor / (2.0 * Math.PI)) * SHOOTER_TICKS_PER_REV;
        } else {
            double interpolatedRPM = interpolateLUT(currentSample, result.effectiveDistanceInches, true);
            result.targetShooterVelocityTPS = (interpolatedRPM / 60.0) * SHOOTER_TICKS_PER_REV * SHOOTER_GEAR_RATIO;
        }

        // Apply Manual RPM Offset
        if (MANUAL_RPM_OFFSET != 0.0) {
            double offsetTPS = (MANUAL_RPM_OFFSET / 60.0) * SHOOTER_TICKS_PER_REV * SHOOTER_GEAR_RATIO;
            result.targetShooterVelocityTPS = Math.max(0.0, result.targetShooterVelocityTPS + offsetTPS);
        }

        result.targetShooterRPM = (result.targetShooterVelocityTPS / (SHOOTER_TICKS_PER_REV * SHOOTER_GEAR_RATIO)) * 60.0;

        if (shooterMotor != null) {
            if (spinUpShooter && result.targetShooterVelocityTPS > 0) {
                shooterMotor.setVelocity(result.targetShooterVelocityTPS);
            } else {
                shooterMotor.setVelocity(0);
            }
            result.actualShooterVelocityTPS = shooterMotor.getVelocity();
            result.actualShooterRPM = (result.actualShooterVelocityTPS / (SHOOTER_TICKS_PER_REV * SHOOTER_GEAR_RATIO)) * 60.0;
        }
        result.shooterVelocityErrorTPS = Math.abs(result.actualShooterVelocityTPS - result.targetShooterVelocityTPS);

        // 8. Basket Entry Angle / Receiving Hemisphere Check
        if (ENABLE_BASKET_ENTRY_ANGLE_CHECK) {
            double robotToBasketAngleDeg = Math.toDegrees(Math.atan2(predTurretY - basket.y, predTurretX - basket.x));
            double entryDevDeg = Math.abs(AngleUnit.normalizeDegrees(robotToBasketAngleDeg - basket.openingDirectionDeg));
            result.isEntryAngleValid = (entryDevDeg <= MAX_ENTRY_ANGLE_DEV_DEG) && basket.isShootable;
        } else {
            result.isEntryAngleValid = basket.isShootable;
        }

        // 9. Dynamic Readiness Check
        double maxLinearSpeed = Math.max(MAX_SOTM_LINEAR_SPEED, MAX_SOTM_LINEAR_SPEED_IN_PER_SEC);

        boolean turretOk = result.turretAngleErrorDeg < TURRET_TOLERANCE_DEG;
        boolean shooterOk = !spinUpShooter || (result.shooterVelocityErrorTPS < VELOCITY_TOLERANCE_TPS);
        boolean distOk = result.effectiveDistanceInches >= MIN_SHOOT_DISTANCE_IN && result.effectiveDistanceInches <= MAX_SHOOT_DISTANCE_IN;
        boolean unclamped = !result.isTurretClamped;
        boolean linSpeedOk = result.robotLinearSpeedInSec < maxLinearSpeed;
        boolean angSpeedOk = Math.abs(result.robotAngularSpeedRadSec) < MAX_SOTM_ANGULAR_SPEED;
        boolean entryOk = result.isEntryAngleValid;

        result.isReadyToShoot = turretOk && shooterOk && distOk && unclamped && linSpeedOk && angSpeedOk && entryOk;

        if (!basket.isShootable) {
            result.blockedReason = "TARGET BASKET NOT SHOOTABLE";
        } else if (!entryOk) {
            result.blockedReason = "BASKET ENTRY ANGLE INVALID (BEHIND RIM)";
        } else if (!distOk) {
            result.blockedReason = "OUT OF DISTANCE RANGE (" + String.format(Locale.US, "%.1f in", result.effectiveDistanceInches) + ")";
        } else if (result.isTurretClamped) {
            result.blockedReason = "TURRET SOFT LIMITED";
        } else if (!turretOk) {
            result.blockedReason = "TURRET AIMING (" + String.format(Locale.US, "%.1f° error", result.turretAngleErrorDeg) + ")";
        } else if (!shooterOk) {
            result.blockedReason = "SHOOTER SPINNING UP (" + String.format(Locale.US, "%.0f TPS error", result.shooterVelocityErrorTPS) + ")";
        } else if (!linSpeedOk) {
            result.blockedReason = "ROBOT DRIVING TOO FAST";
        } else if (!angSpeedOk) {
            result.blockedReason = "ROBOT TURNING TOO FAST";
        } else {
            result.blockedReason = "READY TO FIRE";
        }

        return result;
    }

    public boolean isReadyToShoot() {
        return lastAimResult.isReadyToShoot;
    }

    public AimResult getAimResult() {
        return lastAimResult;
    }

    public void actuateFeedServo(boolean fire) {
        if (feedServo != null) {
            feedServo.setPosition(fire ? FEED_SHOOT_POSITION : FEED_REST_POSITION);
        }
    }

    // =========================================================================
    // HELPER CALCULATIONS & LUTS
    // =========================================================================

    private Basket selectBasket(TargetBasketMode mode, double robotX, double robotY) {
        switch (mode) {
            case BLUE_LEFT:  return BASKET_BLUE_LEFT;
            case BLUE_RIGHT: return BASKET_BLUE_RIGHT;
            case RED_LEFT:   return BASKET_RED_LEFT;
            case RED_RIGHT:  return BASKET_RED_RIGHT;
            case AUTO_NEAREST:
            default:
                if (alliance == AllianceColor.BLUE) {
                    double dLeft = Math.hypot(BASKET_BLUE_LEFT.x - robotX, BASKET_BLUE_LEFT.y - robotY);
                    double dRight = Math.hypot(BASKET_BLUE_RIGHT.x - robotX, BASKET_BLUE_RIGHT.y - robotY);
                    return (dLeft < dRight && BASKET_BLUE_LEFT.isShootable) ? BASKET_BLUE_LEFT : BASKET_BLUE_RIGHT;
                } else {
                    double dLeft = Math.hypot(BASKET_RED_LEFT.x - robotX, BASKET_RED_LEFT.y - robotY);
                    double dRight = Math.hypot(BASKET_RED_RIGHT.x - robotX, BASKET_RED_RIGHT.y - robotY);
                    return (dRight < dLeft && BASKET_RED_RIGHT.isShootable) ? BASKET_RED_RIGHT : BASKET_RED_LEFT;
                }
        }
    }

    private double calculateLaunchVelocityV0(double distanceInches, double basketZInches, SampleType sampleType, double slipFactor) {
        if (USE_PHYSICS_MODEL) {
            double deltaH = basketZInches - SHOOTER_RELEASE_HEIGHT_IN;
            double launchRad = Math.toRadians(LAUNCH_ANGLE_DEG);
            double cosL = Math.cos(launchRad);
            double tanL = Math.tan(launchRad);

            double denom = distanceInches * tanL - deltaH;
            if (denom <= 0) return 0.0; // Unreachable at current launch angle

            double v0Squared = (G_IN_PER_SEC2 * distanceInches * distanceInches) / (2.0 * cosL * cosL * denom);
            return (v0Squared > 0) ? Math.sqrt(v0Squared) : 0.0;
        } else {
            double rpm = interpolateLUT(sampleType, distanceInches, true);
            double vRoller = (rpm * (Math.PI / 30.0)) * ROLLER_RADIUS_IN;
            return vRoller * slipFactor;
        }
    }

    private double interpolateLUT(SampleType sampleType, double distanceInches, boolean returnRPM) {
        List<LUTPoint> lut = (sampleType == SampleType.SMALL) ? smallSampleLUT : largeSampleLUT;
        if (lut.isEmpty()) return 3000.0;
        if (lut.size() == 1) return returnRPM ? lut.get(0).rpm : lut.get(0).timeOfFlightSec;

        if (distanceInches <= lut.get(0).distanceInches) {
            return returnRPM ? lut.get(0).rpm : lut.get(0).timeOfFlightSec;
        }
        if (distanceInches >= lut.get(lut.size() - 1).distanceInches) {
            return returnRPM ? lut.get(lut.size() - 1).rpm : lut.get(lut.size() - 1).timeOfFlightSec;
        }

        for (int i = 0; i < lut.size() - 1; i++) {
            LUTPoint p1 = lut.get(i);
            LUTPoint p2 = lut.get(i + 1);

            if (distanceInches >= p1.distanceInches && distanceInches <= p2.distanceInches) {
                double t = (distanceInches - p1.distanceInches) / (p2.distanceInches - p1.distanceInches);
                double val1 = returnRPM ? p1.rpm : p1.timeOfFlightSec;
                double val2 = returnRPM ? p2.rpm : p2.timeOfFlightSec;
                return val1 + t * (val2 - val1);
            }
        }

        return returnRPM ? lut.get(0).rpm : lut.get(0).timeOfFlightSec;
    }

    private void populateDefaultLUTs() {
        smallSampleLUT.add(new LUTPoint(24.0, 2200.0, 0.22));
        smallSampleLUT.add(new LUTPoint(48.0, 3100.0, 0.38));
        smallSampleLUT.add(new LUTPoint(72.0, 4100.0, 0.54));
        smallSampleLUT.add(new LUTPoint(96.0, 5200.0, 0.72));
        smallSampleLUT.add(new LUTPoint(120.0, 6400.0, 0.92));

        largeSampleLUT.add(new LUTPoint(24.0, 2400.0, 0.24));
        largeSampleLUT.add(new LUTPoint(48.0, 3350.0, 0.41));
        largeSampleLUT.add(new LUTPoint(72.0, 4400.0, 0.58));
        largeSampleLUT.add(new LUTPoint(96.0, 5600.0, 0.76));
        largeSampleLUT.add(new LUTPoint(120.0, 6800.0, 0.98));
    }

    // =========================================================================
    // TELEMETRY & FTC DASHBOARD LIVE FIELD OVERLAY CANVAS DRAWING
    // =========================================================================

    public void sendTelemetry(Telemetry telemetry) {
        AimResult aim = lastAimResult;

        if (telemetry != null) {
            telemetry.addLine("=== BIOBUZZ AUTO TURRET SHOOTER ===");
            telemetry.addData("HEALTH STATUS", isHealthy() ? "HEALTHY" : "CRITICAL UNHEALTHY");
            telemetry.addData("CAN SHOOT?", aim.isReadyToShoot ? "READY TO FIRE!" : "BLOCKED (" + aim.blockedReason + ")");
            telemetry.addLine("----------------------------------------");

            if (aim.targetBasket != null) {
                telemetry.addData("Target Basket", aim.targetBasket.name + String.format(Locale.US, " (%.1f, %.1f, z:%.1f in)", aim.targetBasket.x, aim.targetBasket.y, aim.targetBasket.z));
            }
            telemetry.addData("Turret Field Pos", String.format(Locale.US, "(%.1f, %.1f) in", aim.turretFieldX, aim.turretFieldY));
            telemetry.addData("Virtual Target Pos", String.format(Locale.US, "(%.1f, %.1f) in", aim.virtualBasketX, aim.virtualBasketY));
            telemetry.addData("Static vs Eff Distance", String.format(Locale.US, "%.1f in -> %.1f in", aim.staticDistanceInches, aim.effectiveDistanceInches));
            telemetry.addData("Robot Linear Speed", String.format(Locale.US, "%.1f in/s", aim.robotLinearSpeedInSec));
            telemetry.addData("Est. Time of Flight", String.format(Locale.US, "%.3f s", aim.estimatedTimeOfFlightSec));
            telemetry.addLine("----------------------------------------");

            telemetry.addData("Turret Angle (Tgt / Act)", String.format(Locale.US, "%.1f° / %.1f° (err: %.1f°)", aim.targetTurretAngleDeg, aim.actualTurretAngleDeg, aim.turretAngleErrorDeg));
            telemetry.addData("Turret Soft Clamped?", aim.isTurretClamped ? "YES (CLAMPED)" : "NO");
            telemetry.addData("Basket Entry Angle Valid?", aim.isEntryAngleValid ? "YES (OPEN FRONT)" : "NO (ANGLE INVALID)");
            telemetry.addData("Shooter RPM (Tgt / Act)", String.format(Locale.US, "%.0f / %.0f RPM", aim.targetShooterRPM, aim.actualShooterRPM));
            telemetry.addData("Shooter TPS (Tgt / Act)", String.format(Locale.US, "%.0f / %.0f TPS", aim.targetShooterVelocityTPS, aim.actualShooterVelocityTPS));
            telemetry.addLine("========================================");
        }

        // FTC Dashboard Field Canvas Overlay Drawing & Live Graphing Packet
        try {
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            double robotX = (pinpoint != null) ? pinpoint.getPosition().getX(DistanceUnit.INCH) : INIT_ROBOT_X;
            double robotY = (pinpoint != null) ? pinpoint.getPosition().getY(DistanceUnit.INCH) : INIT_ROBOT_Y;
            double headingRad = (pinpoint != null) ? pinpoint.getPosition().getHeading(AngleUnit.RADIANS) : Math.toRadians(INIT_ROBOT_HEADING_DEG);

            // 1. Robot Pose Circle (Radius 9") and Heading Line
            fieldOverlay.setStroke("blue");
            fieldOverlay.strokeCircle(robotX, robotY, 9.0);
            fieldOverlay.strokeLine(robotX, robotY, robotX + 15.0 * Math.cos(headingRad), robotY + 15.0 * Math.sin(headingRad));

            if (aim.targetBasket != null) {
                // 2. Turret Real-time Aiming Ray to Virtual Target
                fieldOverlay.setStroke("purple");
                fieldOverlay.strokeLine(aim.turretFieldX, aim.turretFieldY, aim.virtualBasketX, aim.virtualBasketY);

                // 3. Actual Basket Circle
                fieldOverlay.setStroke(alliance == AllianceColor.RED ? "red" : "blue");
                fieldOverlay.strokeCircle(aim.targetBasket.x, aim.targetBasket.y, 8.0);

                // 4. Virtual SOTM Target Crosshair/Circle (Green)
                fieldOverlay.setStroke("green");
                fieldOverlay.strokeCircle(aim.virtualBasketX, aim.virtualBasketY, 5.0);
                fieldOverlay.strokeLine(aim.virtualBasketX - 6.0, aim.virtualBasketY, aim.virtualBasketX + 6.0, aim.virtualBasketY);
                fieldOverlay.strokeLine(aim.virtualBasketX, aim.virtualBasketY - 6.0, aim.virtualBasketX, aim.virtualBasketY + 6.0);
            }

            // Real-Time Graph Metrics for FTC Dashboard
            packet.put("targetShooterRPM", aim.targetShooterRPM);
            packet.put("actualShooterRPM", aim.actualShooterRPM);
            packet.put("turretAngleErrorDeg", aim.turretAngleErrorDeg);
            packet.put("effectiveDistanceInches", aim.effectiveDistanceInches);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        } catch (Exception ignored) {}
    }
}
