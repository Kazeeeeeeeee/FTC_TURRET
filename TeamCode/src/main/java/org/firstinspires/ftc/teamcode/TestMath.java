package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * TestMath
 *
 * Pure Java unit test runner for verifying kinematics, trajectory math, conversion formulas,
 * turret angle unwrapping, and calibration algorithms.
 */
public class TestMath {

    public static void main(String[] args) {
        System.out.println("========================================");
        System.out.println("   TURRET SYSTEM PURE-JAVA MATH TESTS   ");
        System.out.println("========================================");

        boolean allPassed = true;

        allPassed &= test1_InchesPerTickConversions();
        allPassed &= test2_CalibrationMath();
        allPassed &= test3_CalculateAimGeometry();
        allPassed &= test4_ChooseTurretAngleLimits();
        allPassed &= test5_BallisticTrajectoryAndAscendingCheck();

        System.out.println("========================================");
        if (allPassed) {
            System.out.println(">>> ALL MATH UNIT TESTS PASSED! <<<");
        } else {
            System.err.println(">>> SOME MATH UNIT TESTS FAILED! <<<");
        }
        System.out.println("========================================");
    }

    public static boolean test1_InchesPerTickConversions() {
        System.out.println("Test 1: INCHES_PER_TICK <-> TicksPerMm Conversions");
        double inchesPerTick = 0.00197945; // ~505.2 ticks/inch
        double ticksPerInch = 1.0 / inchesPerTick;
        double ticksPerMm = ticksPerInch / 25.4;

        double convertedInchesPerTick = 1.0 / (ticksPerMm * 25.4);
        boolean pass = Math.abs(inchesPerTick - convertedInchesPerTick) < 1e-9;
        System.out.printf("  Original: %.8f, Ticks/Mm: %.4f, Converted back: %.8f -> %s\n",
                inchesPerTick, ticksPerMm, convertedInchesPerTick, pass ? "PASS" : "FAIL");
        return pass;
    }

    public static boolean test2_CalibrationMath() {
        System.out.println("Test 2: Push Test Calibration & Correction Formula");
        double pushDistance = 96.0;
        long deltaTicks = 48500L;
        double calculatedInchesPerTick = InchesPerTickCalibrationOpMode.calculateInchesPerTick(pushDistance, deltaTicks);

        double oldTicksPerInch = 500.0;
        double reportedDist = 98.4;
        double actualDist = 96.0;
        double correctedTicksPerInch = InchesPerTickCalibrationOpMode.calculateCorrectedTicksPerUnit(oldTicksPerInch, reportedDist, actualDist);

        boolean pass = (calculatedInchesPerTick > 0) && (Math.abs(correctedTicksPerInch - (500.0 * 98.4 / 96.0)) < 1e-6);
        System.out.printf("  Push 96in / 48500 ticks -> %.8f in/tick\n", calculatedInchesPerTick);
        System.out.printf("  Correction 500 * 98.4/96 -> %.4f ticks/in -> %s\n", correctedTicksPerInch, pass ? "PASS" : "FAIL");
        return pass;
    }

    public static boolean test3_CalculateAimGeometry() {
        System.out.println("Test 3: CalculateAim Kinematics & Geometry");
        TurretAimCalculator calc = new TurretAimCalculator();
        calc.basketXInches = 48.0;
        calc.basketYInches = 12.0;
        calc.targetHeightInches = 46.5;
        calc.turretExitHeightInches = 12.0;
        calc.exitForwardOffsetInches = 2.0;

        // Robot at (0, 0) with Heading 0 deg
        Pose2D pose0 = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        TurretAimCalculator.AimResult aim0 = calc.calculateAim(pose0);

        double expectedFieldAngle = Math.toDegrees(Math.atan2(12.0, 48.0)); // ~14.03 deg
        double expectedRawDist = Math.hypot(48.0, 12.0); // ~49.477 in
        double expectedSubtractedDist = expectedRawDist - 2.0; // ~47.477 in

        boolean passAngle0 = Math.abs(aim0.turretFieldAngleDegrees - expectedFieldAngle) < 1e-3;
        boolean passDist0 = Math.abs(aim0.horizontalDistanceInches - expectedSubtractedDist) < 1e-3;

        // Robot at (0, 0) with Heading 90 deg
        Pose2D pose90 = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90);
        TurretAimCalculator.AimResult aim90 = calc.calculateAim(pose90);
        double expectedRel90 = TurretAimCalculator.normalizeAngleDegrees(expectedFieldAngle - 90.0); // ~ -75.96 deg

        boolean passRel90 = Math.abs(aim90.turretRelativeAngleDegrees - expectedRel90) < 1e-3;

        boolean pass = passAngle0 && passDist0 && passRel90;
        System.out.printf("  Field Angle: %.2f deg (expected %.2f)\n", aim0.turretFieldAngleDegrees, expectedFieldAngle);
        System.out.printf("  Horizontal Dist (Subtracted): %.2f in (expected %.2f)\n", aim0.horizontalDistanceInches, expectedSubtractedDist);
        System.out.printf("  Rel Angle at Heading 90: %.2f deg (expected %.2f) -> %s\n", aim90.turretRelativeAngleDegrees, expectedRel90, pass ? "PASS" : "FAIL");
        return pass;
    }

    public static boolean test4_ChooseTurretAngleLimits() {
        System.out.println("Test 4: chooseTurretAngle Limits & Unwrapping");
        TurretAimCalculator calc = new TurretAimCalculator();
        calc.minTurretAngleDeg = -135.0;
        calc.maxTurretAngleDeg = 135.0;
        calc.servoTotalRangeDegrees = 360.0;
        calc.lastCommandedTurretAngleDeg = 0.0;

        double angle90 = calc.chooseTurretAngle(90.0);
        double angle170 = calc.chooseTurretAngle(170.0);
        double angleNeg170 = calc.chooseTurretAngle(-170.0);

        boolean pass90 = (Math.abs(angle90 - 90.0) < 1e-6);
        boolean pass170 = Double.isNaN(angle170);
        boolean passNeg170 = Double.isNaN(angleNeg170);

        // Wider limits test for unwrapping
        calc.minTurretAngleDeg = -270.0;
        calc.maxTurretAngleDeg = 270.0;
        calc.lastCommandedTurretAngleDeg = 170.0;
        double angleUnwrap = calc.chooseTurretAngle(-175.0); // candidates: -175, 185. 185 is closer to 170!

        boolean passUnwrap = (Math.abs(angleUnwrap - 185.0) < 1e-6);

        boolean pass = pass90 && pass170 && passNeg170 && passUnwrap;
        System.out.printf("  desired 90 -> %.1f, desired 170 -> %s, desired -170 -> %s\n", angle90, angle170, angleNeg170);
        System.out.printf("  last=170, desired -175 -> %.1f (expected 185.0) -> %s\n", angleUnwrap, pass ? "PASS" : "FAIL");
        return pass;
    }

    public static boolean test5_BallisticTrajectoryAndAscendingCheck() {
        System.out.println("Test 5: Ballistic Trajectory Verification & Descending Check");
        TurretAimCalculator calc = new TurretAimCalculator();
        calc.basketXInches = 60.0;
        calc.basketYInches = 0.0;
        calc.targetHeightInches = 43.0 + 3.5; // 46.5 in
        calc.turretExitHeightInches = 12.0;    // deltaZ = 34.5 in
        calc.launchAngleDegrees = 50.0;
        calc.exitForwardOffsetInches = 0.0;
        calc.flywheelEfficiency = 0.65;
        calc.powerMultiplier = 1.0;
        calc.maxMotorRPM = 6000.0;

        Pose2D pose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        TurretAimCalculator.AimResult aim = calc.calculateAim(pose);

        // Verify trajectory equation: z(t) = v0*sin(theta)*t - 0.5*g*t^2 = deltaZ
        double rad = Math.toRadians(50.0);
        double t = aim.horizontalDistanceInches / (aim.targetLaunchVelocityInSec * Math.cos(rad));
        double zCalculated = aim.targetLaunchVelocityInSec * Math.sin(rad) * t - 0.5 * TurretAimCalculator.GRAVITY_INCHES_PER_SEC2 * t * t;
        double deltaZExpected = 46.5 - 12.0; // 34.5 in

        boolean passTraj = Math.abs(zCalculated - deltaZExpected) < 1e-3;
        boolean passVy = aim.descending && (aim.vyAtTargetInSec < 0.0);

        // Short-range ascending check: target very close and high
        calc.basketXInches = 5.0; // Very close
        TurretAimCalculator.AimResult aimShort = calc.calculateAim(pose);
        boolean passAscendingInvalid = !aimShort.shotValid;

        boolean pass = passTraj && passVy && passAscendingInvalid;
        System.out.printf("  60in Shot: v0=%.2f in/s, z_calc=%.2f in (expected %.2f), vy=%.2f in/s -> descending=%b, shotValid=%b\n",
                aim.targetLaunchVelocityInSec, zCalculated, deltaZExpected, aim.vyAtTargetInSec, aim.descending, aim.shotValid);
        System.out.printf("  Short Range Shot: targetInReach=%b, descending=%b, shotValid=%b -> %s\n",
                aimShort.targetInReach, aimShort.descending, aimShort.shotValid, pass ? "PASS" : "FAIL");
        return pass;
    }
}
