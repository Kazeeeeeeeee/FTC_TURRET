package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * TurretAimCalculator
 *
 * Handles 2D/3D kinematic and ballistic calculations for auto-aiming a robot turret
 * and shooter flywheel at a target basket on an FTC field.
 *
 * Uses field coordinate system where:
 *   - +X is Forward
 *   - +Y is Left
 *   - 0 degrees heading points along +X axis
 */
public class TurretAimCalculator {

    // =========================================================================
    // CONFIGURABLE PARAMETERS
    // =========================================================================

    /** Radius of the flywheel (distance from motor center axis to wheel edge) in INCHES */
    public double flywheelRadiusInches = 2.0;

    /** Turret mounting offset relative to robot center in INCHES (+X = forward, +Y = left) */
    public double turretOffsetXInches = 0.0;
    public double turretOffsetYInches = 0.0;

    /** Height of the turret launch pivot point above the ground in INCHES */
    public double turretHeightInches = 12.0;

    /** 2D Target Basket Field Coordinate (X, Y) in INCHES */
    public double basketXInches = 0.0;
    public double basketYInches = 0.0;

    /** Basket rim height above ground in INCHES */
    public double basketHeightInches = 43.0; // Standard FTC high basket height is ~43 inches

    /** Launch ramp angle relative to horizontal plane in DEGREES */
    public double launchAngleDegrees = 50.0;

    /** Energy transfer efficiency from flywheel surface to sample (0.0 to 1.0) */
    public double flywheelEfficiency = 0.65;

    /** Tunable power multiplier (fine-tuned live via FTC Dashboard to scale flywheel output) */
    public double powerMultiplier = 1.0;

    /** Max motor RPM at 1.0 power (e.g., 6000 RPM for 1:1 bare motor) */
    public double maxMotorRPM = 6000.0;

    /** 
     * Servo motion range specs:
     * servoCenterPosition (default 0.5) MUST correspond to 0 deg relative angle 
     * (the shooter facing DIRECTLY FORWARD along the front of the robot).
     */
    public double servoCenterPosition = 0.5;   // Position where shooter faces DIRECTLY FORWARD
    public double servoTotalRangeDegrees = 180.0; // Total angular sweep of servo (0.0 to 1.0)
    public boolean reverseServoDirection = false; // Set to true if servo turns opposite direction

    // Gravitational constant in inches/s^2 (9.81 m/s^2 * 39.3701 = 386.09 inches/s^2)
    private static final double GRAVITY_INCHES_PER_SEC2 = 386.09;

    // =========================================================================
    // CALCULATION RESULTS DATA CLASS
    // =========================================================================

    public static class AimResult {
        public double turretFieldAngleDegrees;  // Target field angle from turret to basket
        public double turretRelativeAngleDegrees; // Angle turret must turn relative to robot heading
        public double servoPosition;             // Position to command turret servo (0.0 to 1.0)
        public double horizontalDistanceInches;  // 2D distance from turret to basket
        public double targetLaunchVelocityInSec; // Required initial projectile velocity (in/s)
        public double requiredFlywheelRPM;       // Target flywheel motor RPM
        public double motorPower;                // Scaled motor power command (0.0 to 1.0)
        public boolean targetInReach;            // True if ballistic solution exists
        public String statusMessage;             // Descriptive status string for telemetry
    }

    // =========================================================================
    // CALCULATION METHODS
    // =========================================================================

    /**
     * Calculates the complete aiming solution based on current robot pose.
     *
     * @param robotPose Current robot position and heading from odometry (Pose2D)
     * @return AimResult object containing calculated servo positions, motor power, and distance.
     */
    public AimResult calculateAim(Pose2D robotPose) {
        AimResult result = new AimResult();

        double robotX = robotPose.getX(DistanceUnit.INCH);
        double robotY = robotPose.getY(DistanceUnit.INCH);
        double robotHeadingRad = robotPose.getHeading(AngleUnit.RADIANS);
        double robotHeadingDeg = robotPose.getHeading(AngleUnit.DEGREES);

        // 1. Calculate Turret position on field accounting for robot orientation and turret offset
        double turretX = robotX + turretOffsetXInches * Math.cos(robotHeadingRad) - turretOffsetYInches * Math.sin(robotHeadingRad);
        double turretY = robotY + turretOffsetXInches * Math.sin(robotHeadingRad) + turretOffsetYInches * Math.cos(robotHeadingRad);

        // 2. Vector from turret to basket
        double deltaX = basketXInches - turretX;
        double deltaY = basketYInches - turretY;
        double deltaZ = basketHeightInches - turretHeightInches;

        // 3. 2D Horizontal distance
        result.horizontalDistanceInches = Math.hypot(deltaX, deltaY);

        // Avoid division by zero when directly under target
        if (result.horizontalDistanceInches < 0.1) {
            result.horizontalDistanceInches = 0.1;
        }

        // 4. Turret Field Angle and Relative Angle
        double fieldAngleRad = Math.atan2(deltaY, deltaX);
        result.turretFieldAngleDegrees = Math.toDegrees(fieldAngleRad);

        // Relative angle turret needs to turn from robot's forward vector (+X direction, 0 deg)
        double relativeAngleDeg = result.turretFieldAngleDegrees - robotHeadingDeg;
        result.turretRelativeAngleDegrees = normalizeAngleDegrees(relativeAngleDeg);

        // 5. Servo Position calculation (0.0 deg relative angle -> facing directly forward)
        result.servoPosition = calculateServoPosition(result.turretRelativeAngleDegrees);

        // 6. Ballistic trajectory launch velocity v0 calculation
        double launchAngleRad = Math.toRadians(launchAngleDegrees);
        double cosAngle = Math.cos(launchAngleRad);
        double tanAngle = Math.tan(launchAngleRad);

        double d = result.horizontalDistanceInches;

        // Denominator of projectile motion equation: d * tan(theta) - deltaZ
        double denominator = d * tanAngle - deltaZ;

        if (denominator <= 0) {
            // Target is unreachable at current launch angle and height
            result.targetInReach = false;
            result.targetLaunchVelocityInSec = 0.0;
            result.requiredFlywheelRPM = 0.0;
            result.motorPower = 0.0;
            result.statusMessage = "OUT OF RANGE (Target too close/high for launch angle)";
            return result;
        }

        // Formula: v0^2 = (g * d^2) / (2 * cos^2(theta) * (d * tan(theta) - deltaZ))
        double v0Squared = (GRAVITY_INCHES_PER_SEC2 * d * d) / (2.0 * cosAngle * cosAngle * denominator);

        if (v0Squared < 0) {
            result.targetInReach = false;
            result.targetLaunchVelocityInSec = 0.0;
            result.requiredFlywheelRPM = 0.0;
            result.motorPower = 0.0;
            result.statusMessage = "NO BALLISTIC SOLUTION";
            return result;
        }

        result.targetLaunchVelocityInSec = Math.sqrt(v0Squared);

        // 7. Calculate required Flywheel RPM
        // Linear wheel edge speed required = v0 / efficiency
        double requiredEdgeSpeedInSec = result.targetLaunchVelocityInSec / flywheelEfficiency;

        // Angular velocity omega (rad/s) = v_edge / radius
        double omegaRadPerSec = requiredEdgeSpeedInSec / flywheelRadiusInches;

        // Convert rad/sec to RPM: RPM = omega * (60 / 2pi) = omega * (30 / pi)
        result.requiredFlywheelRPM = omegaRadPerSec * (30.0 / Math.PI);

        // 8. Convert RPM to Motor Power ratio (0.0 to 1.0) scaled by powerMultiplier
        double basePower = result.requiredFlywheelRPM / maxMotorRPM;
        result.motorPower = Math.min(1.0, Math.max(0.0, basePower * powerMultiplier));

        result.targetInReach = true;
        result.statusMessage = "AIM LOCKED";

        return result;
    }

    /**
     * Calculates aiming solution with a custom/live power multiplier.
     */
    public AimResult calculateAim(Pose2D robotPose, double customPowerMultiplier) {
        double tempMultiplier = this.powerMultiplier;
        this.powerMultiplier = customPowerMultiplier;
        AimResult result = calculateAim(robotPose);
        this.powerMultiplier = tempMultiplier;
        return result;
    }

    /**
     * Returns the servo position required to point the shooter DIRECTLY FORWARD relative to the robot.
     */
    public double getForwardServoPosition() {
        return servoCenterPosition;
    }

    /**
     * Converts relative turret angle in degrees to servo position [0.0, 1.0].
     * Relative angle 0.0 degrees always maps to facing DIRECTLY FORWARD along the robot.
     */
    public double calculateServoPosition(double relativeAngleDegrees) {
        // Position offset from center: angle / total_sweep_range
        double offset = relativeAngleDegrees / servoTotalRangeDegrees;
        if (reverseServoDirection) {
            offset = -offset;
        }
        double pos = servoCenterPosition + offset;
        return Math.min(1.0, Math.max(0.0, pos));
    }

    /**
     * Normalizes angle into [-180, +180] degrees range.
     */
    private double normalizeAngleDegrees(double degrees) {
        while (degrees > 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;
        return degrees;
    }
}
