package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * TurretAimCalculator
 *
 * Handles 2D/3D kinematic and ballistic calculations for auto-aiming a robot turret
 * and shooter flywheel at a target basket on an FTC field.
 */
public class TurretAimCalculator {

    // =========================================================================
    // CONFIGURABLE PARAMETERS (Read from TurretFieldConfig)
    // =========================================================================

    /** Radius of the flywheel (distance from motor center axis to wheel edge) in INCHES */
    public double flywheelRadiusInches = TurretFieldConfig.FLYWHEEL_RADIUS_INCHES;

    /** Turret mounting offset relative to robot center in INCHES (+X = forward, +Y = left) */
    public double turretOffsetXInches = TurretFieldConfig.TURRET_OFFSET_X_INCHES;
    public double turretOffsetYInches = TurretFieldConfig.TURRET_OFFSET_Y_INCHES;

    /** Height of the turret launch pivot point above the ground in INCHES */
    public double turretExitHeightInches = TurretFieldConfig.TURRET_EXIT_HEIGHT_INCHES;

    /** Horizontal forward offset of ball exit point ahead of turret pivot axis in INCHES */
    public double exitForwardOffsetInches = TurretFieldConfig.EXIT_FORWARD_OFFSET_INCHES;

    /** 2D Target Basket Field Coordinate (X, Y) in INCHES */
    public double basketXInches = TurretFieldConfig.getSelectedProfile().basketDxInches;
    public double basketYInches = TurretFieldConfig.getSelectedProfile().basketDyInches;

    /** Target rim + clearance height above ground in INCHES */
    public double targetHeightInches = TurretFieldConfig.getTargetHeightInches();

    /** Launch ramp angle relative to horizontal plane in DEGREES */
    public double launchAngleDegrees = TurretFieldConfig.LAUNCH_ANGLE_DEGREES;

    /** Energy transfer efficiency from flywheel surface to sample (0.0 to 1.0) */
    public double flywheelEfficiency = 0.65;

    /** Tunable power multiplier (fine-tuned live via FTC Dashboard to scale flywheel output) */
    public double powerMultiplier = 1.0;

    /** Max motor RPM at 1.0 power (e.g., 6000 RPM for 1:1 bare motor) */
    public double maxMotorRPM = 6000.0;

    /** Servo motion range specs */
    public double servoCenterPosition = 0.5;   // Position where shooter faces DIRECTLY FORWARD
    public double servoTotalRangeDegrees = TurretFieldConfig.SERVO_TOTAL_RANGE_DEGREES; // Total angular sweep of servo for [0.0, 1.0]
    public boolean reverseServoDirection = false; // Set to true if servo turns opposite direction

    /** Turret hard-stop and cable angle limits in DEGREES */
    public double minTurretAngleDeg = TurretFieldConfig.MIN_TURRET_ANGLE_DEG;
    public double maxTurretAngleDeg = TurretFieldConfig.MAX_TURRET_ANGLE_DEG;

    /** Memory of last commanded turret angle to prevent 360-deg unwrapping jumps */
    public double lastCommandedTurretAngleDeg = 0.0;

    // Gravitational constant in inches/s^2 (9.81 m/s^2 * 39.3701 = 386.09 inches/s^2)
    public static final double GRAVITY_INCHES_PER_SEC2 = 386.09;

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
        public double vyAtTargetInSec;           // Vertical velocity of projectile at target height
        public boolean targetInReach;            // True if ballistic solution exists
        public boolean descending;               // True if projectile is descending at target (vy < 0)
        public boolean powerSaturated;           // True if requested motor power exceeds 1.0
        public boolean turretAtLimit;            // True if target angle exceeds hard-stop limits
        public boolean shotValid;                // True ONLY if targetInReach && descending && !powerSaturated && !turretAtLimit
        public String statusMessage;             // Descriptive status string for telemetry
    }

    // =========================================================================
    // CALCULATION METHODS
    // =========================================================================

    /** Resets last commanded turret angle memory to 0.0 (directly forward) */
    public void resetTurretAngleMemory() {
        lastCommandedTurretAngleDeg = 0.0;
    }

    /**
     * Considers desired relative angle, desired + 360, and desired - 360.
     * Keeps only candidates inside [minTurretAngleDeg, maxTurretAngleDeg] AND [-servoTotalRangeDegrees/2, +servoTotalRangeDegrees/2].
     * Picks the candidate closest to lastCommandedTurretAngleDeg.
     * Returns Double.NaN if no valid candidate exists.
     */
    public double chooseTurretAngle(double desiredDeg) {
        double normDesired = normalizeAngleDegrees(desiredDeg);
        double[] candidates = new double[]{ normDesired, normDesired + 360.0, normDesired - 360.0 };

        double maxServoAngle = Math.abs(servoTotalRangeDegrees) / 2.0;
        double effectiveMin = Math.max(minTurretAngleDeg, -maxServoAngle);
        double effectiveMax = Math.min(maxTurretAngleDeg, maxServoAngle);

        double bestCandidate = Double.NaN;
        double minDistance = Double.MAX_VALUE;

        for (double c : candidates) {
            if (c >= effectiveMin && c <= effectiveMax) {
                double dist = Math.abs(c - lastCommandedTurretAngleDeg);
                if (dist < minDistance) {
                    minDistance = dist;
                    bestCandidate = c;
                }
            }
        }

        return bestCandidate;
    }

    /**
     * Calculates the complete aiming solution based on current robot pose.
     *
     * @param robotPose Current robot position and heading from odometry (Pose2D)
     * @return AimResult object containing calculated servo positions, motor power, distance, and validity.
     */
    public AimResult calculateAim(Pose2D robotPose) {
        return calculateAim(robotPose, powerMultiplier);
    }

    /**
     * Calculates aiming solution with a custom power multiplier.
     */
    public AimResult calculateAim(Pose2D robotPose, double customPowerMultiplier) {
        AimResult result = new AimResult();

        double robotX = robotPose.getX(DistanceUnit.INCH);
        double robotY = robotPose.getY(DistanceUnit.INCH);
        double robotHeadingRad = robotPose.getHeading(AngleUnit.RADIANS);
        double robotHeadingDeg = robotPose.getHeading(AngleUnit.DEGREES);

        // 1. Calculate Turret position on field accounting for robot orientation and turret offset
        double turretX = robotX + turretOffsetXInches * Math.cos(robotHeadingRad) - turretOffsetYInches * Math.sin(robotHeadingRad);
        double turretY = robotY + turretOffsetXInches * Math.sin(robotHeadingRad) + turretOffsetYInches * Math.cos(robotHeadingRad);

        // 2. Vector from turret to basket center
        double deltaX = basketXInches - turretX;
        double deltaY = basketYInches - turretY;
        double deltaZ = targetHeightInches - turretExitHeightInches;

        // 3. 2D Horizontal distance subtraction of EXIT_FORWARD_OFFSET_INCHES
        double rawDistance = Math.hypot(deltaX, deltaY);
        result.horizontalDistanceInches = Math.max(0.1, rawDistance - exitForwardOffsetInches);

        // 4. Turret Field Angle and Relative Angle
        double fieldAngleRad = Math.atan2(deltaY, deltaX);
        result.turretFieldAngleDegrees = Math.toDegrees(fieldAngleRad);

        double rawRelativeAngleDeg = result.turretFieldAngleDegrees - robotHeadingDeg;

        // 5. Unwrapping & Angle limit check
        double chosenAngle = chooseTurretAngle(rawRelativeAngleDeg);
        if (Double.isNaN(chosenAngle)) {
            result.turretAtLimit = true;
            double clamped = Math.min(maxTurretAngleDeg, Math.max(minTurretAngleDeg, normalizeAngleDegrees(rawRelativeAngleDeg)));
            result.turretRelativeAngleDegrees = clamped;
        } else {
            result.turretAtLimit = false;
            result.turretRelativeAngleDegrees = chosenAngle;
            lastCommandedTurretAngleDeg = chosenAngle;
        }

        result.servoPosition = calculateServoPosition(result.turretRelativeAngleDegrees);

        // 6. Ballistic trajectory launch velocity v0 calculation
        double launchAngleRad = Math.toRadians(launchAngleDegrees);
        double cosAngle = Math.cos(launchAngleRad);
        double tanAngle = Math.tan(launchAngleRad);

        double d = result.horizontalDistanceInches;
        double denominator = d * tanAngle - deltaZ;

        if (denominator <= 0) {
            result.targetInReach = false;
            result.descending = false;
            result.powerSaturated = false;
            result.shotValid = false;
            result.targetLaunchVelocityInSec = 0.0;
            result.requiredFlywheelRPM = 0.0;
            result.motorPower = 0.0;
            result.statusMessage = result.turretAtLimit ? "TURRET AT LIMIT" : "NO BALLISTIC SOLUTION";
            return result;
        }

        // Formula: v0^2 = (g * d^2) / (2 * cos^2(theta) * (d * tan(theta) - deltaZ))
        double v0Squared = (GRAVITY_INCHES_PER_SEC2 * d * d) / (2.0 * cosAngle * cosAngle * denominator);

        if (v0Squared < 0) {
            result.targetInReach = false;
            result.descending = false;
            result.powerSaturated = false;
            result.shotValid = false;
            result.targetLaunchVelocityInSec = 0.0;
            result.requiredFlywheelRPM = 0.0;
            result.motorPower = 0.0;
            result.statusMessage = result.turretAtLimit ? "TURRET AT LIMIT" : "NO BALLISTIC SOLUTION";
            return result;
        }

        result.targetInReach = true;
        result.targetLaunchVelocityInSec = Math.sqrt(v0Squared);

        // Compute vertical velocity at target: vy = v0 * sin(theta) - g * d / (v0 * cos(theta))
        double timeOfFlight = d / (result.targetLaunchVelocityInSec * cosAngle);
        result.vyAtTargetInSec = result.targetLaunchVelocityInSec * Math.sin(launchAngleRad) - GRAVITY_INCHES_PER_SEC2 * timeOfFlight;
        result.descending = (result.vyAtTargetInSec < 0.0);

        // 7. Calculate required Flywheel RPM
        double requiredEdgeSpeedInSec = result.targetLaunchVelocityInSec / flywheelEfficiency;
        double omegaRadPerSec = requiredEdgeSpeedInSec / flywheelRadiusInches;
        result.requiredFlywheelRPM = omegaRadPerSec * (30.0 / Math.PI);

        // 8. Motor Power & Saturation check
        double rawPowerNeeded = (result.requiredFlywheelRPM / maxMotorRPM) * customPowerMultiplier;
        result.powerSaturated = (rawPowerNeeded > 1.0);
        result.motorPower = Math.min(1.0, Math.max(0.0, rawPowerNeeded));

        // 9. Shot Validity: ONLY valid if solution exists, trajectory is descending, power is not saturated, and turret is not at limit!
        result.shotValid = result.targetInReach && result.descending && !result.powerSaturated && !result.turretAtLimit;

        // Accurate Status Message
        if (result.turretAtLimit) {
            result.statusMessage = "TURRET AT LIMIT";
        } else if (!result.descending) {
            result.statusMessage = "DESCENT INVALID (ASCENDING)";
        } else if (result.powerSaturated) {
            result.statusMessage = "POWER SATURATED";
        } else if (result.shotValid) {
            result.statusMessage = "AIM LOCKED";
        } else {
            result.statusMessage = "NO BALLISTIC SOLUTION";
        }

        return result;
    }

    /** Returns the servo position required to point the shooter DIRECTLY FORWARD relative to the robot */
    public double getForwardServoPosition() {
        return servoCenterPosition;
    }

    /** Converts relative turret angle in degrees to servo position [0.0, 1.0] */
    public double calculateServoPosition(double relativeAngleDegrees) {
        double offset = relativeAngleDegrees / servoTotalRangeDegrees;
        if (reverseServoDirection) {
            offset = -offset;
        }
        double pos = servoCenterPosition + offset;
        return Math.min(1.0, Math.max(0.0, pos));
    }

    /** Normalizes angle into [-180, +180] degrees range */
    public static double normalizeAngleDegrees(double degrees) {
        double d = degrees;
        while (d > 180.0) d -= 360.0;
        while (d < -180.0) d += 360.0;
        return d;
    }
}
