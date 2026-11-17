package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * InchesPerTickCalibrationOpMode
 *
 * Driver-Station OpMode for calibrating exact INCHES_PER_TICK for goBILDA Pinpoint Odometry pods.
 *
 * WORKFLOW:
 * 1. Forward Push Test (X Pod): Place robot on seam line. Press [A] to capture start ticks.
 *    Push robot straight forward exactly 96 inches (4 tile seams). Press [B] to capture.
 *    Perform 3 trials.
 * 2. Strafe Push Test (Y Pod): Repeat for leftward push. Perform 3 trials.
 * 3. Shows individual trials, mean, spread, and warnings if spread > 0.5% or X/Y difference > 1%.
 * 4. Saves tuned INCHES_PER_TICK to settings file "inches_per_tick.txt".
 */
@TeleOp(name = "Inches-Per-Tick Calibration", group = "Calibration")
public class InchesPerTickCalibrationOpMode extends LinearOpMode {

    public static final double PUSH_DISTANCE_INCHES = 96.0; // 4 tiles = 96 inches
    private GoBildaPinpointDriver pinpoint;

    private final List<Double> xTrials = new ArrayList<>();
    private final List<Double> yTrials = new ArrayList<>();

    public static final String FILE_NAME = "inches_per_tick.txt";

    /** Pure math correction formula for unit testing: new_ticks_per_unit = old_ticks_per_unit * reported / actual */
    public static double calculateCorrectedTicksPerUnit(double oldTicksPerUnit, double reportedDist, double actualDist) {
        if (actualDist <= 0) return oldTicksPerUnit;
        return oldTicksPerUnit * (reportedDist / actualDist);
    }

    /** Pure math formula for computing inches per tick from push distance and tick delta */
    public static double calculateInchesPerTick(double pushDistanceInches, long deltaTicks) {
        if (deltaTicks == 0) return TurretFieldConfig.INCHES_PER_TICK;
        return pushDistanceInches / Math.abs((double) deltaTicks);
    }

    @Override
    public void runOpMode() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(TurretFieldConfig.PINPOINT_X_OFFSET_MM, TurretFieldConfig.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);

        if (TurretFieldConfig.USE_CUSTOM_INCHES_PER_TICK) {
            double ticksPerInch = 1.0 / TurretFieldConfig.INCHES_PER_TICK;
            pinpoint.setEncoderResolution(ticksPerInch, DistanceUnit.INCH);
        } else {
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        }

        pinpoint.resetPosAndIMU();

        telemetry.addLine("=== INCHES-PER-TICK CALIBRATION ===");
        telemetry.addLine("Push Distance: " + PUSH_DISTANCE_INCHES + " inches (4 tiles)");
        telemetry.addLine("Press START to begin X-pod (Forward) trials.");
        telemetry.update();

        waitForStart();

        // PHASE 1: FORWARD TRIALS (X POD)
        runPodTrials("FORWARD (X Pod)", true);

        // PHASE 2: STRAFE TRIALS (Y POD)
        runPodTrials("STRAFE (Y Pod)", false);

        // RESULTS EVALUATION
        double xMean = calculateMean(xTrials);
        double yMean = calculateMean(yTrials);
        double overallMean = (xMean + yMean) / 2.0;

        double xSpreadPct = calculateSpreadPercent(xTrials, xMean);
        double ySpreadPct = calculateSpreadPercent(yTrials, yMean);
        double xyDiffPct = Math.abs(xMean - yMean) / overallMean * 100.0;

        // Save to file
        saveInchesPerTick(overallMean);

        // PHASE 3: SUMMARY & VERIFICATION
        while (opModeIsActive()) {
            pinpoint.update();
            double reportedX = pinpoint.getPosition().getX(DistanceUnit.INCH);
            double reportedY = pinpoint.getPosition().getY(DistanceUnit.INCH);

            telemetry.addLine("========================================");
            telemetry.addLine("    INCHES-PER-TICK CALIBRATION RESULTS ");
            telemetry.addLine("========================================");
            telemetry.addData("X Pod Mean", String.format(Locale.US, "%.8f in/tick (Spread: %.2f%%)", xMean, xSpreadPct));
            telemetry.addData("Y Pod Mean", String.format(Locale.US, "%.8f in/tick (Spread: %.2f%%)", yMean, ySpreadPct));
            telemetry.addData("RECOMMENDED OVERALL MEAN", String.format(Locale.US, "%.8f in/tick", overallMean));
            telemetry.addLine("----------------------------------------");

            if (xSpreadPct > 0.5 || ySpreadPct > 0.5) {
                telemetry.addLine("⚠️ WARNING: Trial spread exceeds 0.5%! Check pod mounting/alignment.");
            }
            if (xyDiffPct > 1.0) {
                telemetry.addLine("⚠️ WARNING: X and Y pods differ by > 1.0%! Check wheel diameters/contact.");
            }

            telemetry.addLine("----------------------------------------");
            telemetry.addLine("PASTE INTO TurretFieldConfig.java:");
            telemetry.addLine(String.format(Locale.US, "public static double INCHES_PER_TICK = %.8f;", overallMean));
            telemetry.addLine("public static boolean INCHES_PER_TICK_CONFIRMED = true;");
            telemetry.addLine("----------------------------------------");

            telemetry.addLine("VERIFICATION PUSH:");
            telemetry.addData("Reported X (Target: 96.0 in)", String.format(Locale.US, "%.2f in", reportedX));
            telemetry.addData("Reported Y (Target: 96.0 in)", String.format(Locale.US, "%.2f in", reportedY));
            telemetry.addLine("Press [A] to re-zero position for verification push.");

            if (gamepad1.a) {
                pinpoint.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(DistanceUnit.INCH, 0, 0, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES, 0));
            }

            telemetry.addLine("========================================");
            telemetry.update();
            sleep(20);
        }
    }

    private void runPodTrials(String podName, boolean isX) {
        List<Double> trialsList = isX ? xTrials : yTrials;

        for (int trial = 1; trial <= 3; trial++) {
            telemetry.clearAll();
            telemetry.addLine("=== " + podName + " - TRIAL " + trial + " OF 3 ===");
            telemetry.addLine("1. Line up robot flush on start mark.");
            telemetry.addLine("2. Press [A] on Gamepad 1 to CAPTURE START baseline.");
            telemetry.update();

            while (opModeIsActive() && !gamepad1.a) {
                sleep(10);
            }
            while (opModeIsActive() && gamepad1.a) {
                sleep(10); // Wait for release
            }

            pinpoint.update();
            long startTicks = isX ? pinpoint.getEncoderX() : pinpoint.getEncoderY();

            telemetry.clearAll();
            telemetry.addLine("=== " + podName + " - TRIAL " + trial + " RUNNING ===");
            telemetry.addLine("Baseline Ticks Captured: " + startTicks);
            telemetry.addLine("NOW push robot straight " + (isX ? "FORWARD" : "LEFT") + " exactly " + PUSH_DISTANCE_INCHES + " inches (4 tiles).");
            telemetry.addLine("Press [B] on Gamepad 1 to CAPTURE END position.");
            telemetry.update();

            while (opModeIsActive() && !gamepad1.b) {
                pinpoint.update();
                long curTicks = isX ? pinpoint.getEncoderX() : pinpoint.getEncoderY();
                long delta = Math.abs(curTicks - startTicks);
                double approxInches = delta * TurretFieldConfig.INCHES_PER_TICK;

                telemetry.addData("Live Delta Ticks", delta);
                telemetry.addData("Approx Inches", String.format(Locale.US, "%.2f in", approxInches));
                telemetry.update();
                sleep(20);
            }
            while (opModeIsActive() && gamepad1.b) {
                sleep(10); // Wait for release
            }

            pinpoint.update();
            long endTicks = isX ? pinpoint.getEncoderX() : pinpoint.getEncoderY();
            long deltaTicks = Math.abs(endTicks - startTicks);

            double trialInchesPerTick = calculateInchesPerTick(PUSH_DISTANCE_INCHES, deltaTicks);
            trialsList.add(trialInchesPerTick);

            telemetry.clearAll();
            telemetry.addLine("=== TRIAL " + trial + " COMPLETE ===");
            telemetry.addData("Delta Ticks", deltaTicks);
            telemetry.addData("Measured INCHES_PER_TICK", String.format(Locale.US, "%.8f", trialInchesPerTick));
            telemetry.addLine("Press [A] to continue to next trial.");
            telemetry.update();

            while (opModeIsActive() && !gamepad1.a) {
                sleep(10);
            }
            while (opModeIsActive() && gamepad1.a) {
                sleep(10);
            }
        }
    }

    private double calculateMean(List<Double> values) {
        if (values.isEmpty()) return TurretFieldConfig.INCHES_PER_TICK;
        double sum = 0.0;
        for (double v : values) sum += v;
        return sum / values.size();
    }

    private double calculateSpreadPercent(List<Double> values, double mean) {
        if (values.isEmpty() || mean == 0) return 0.0;
        double min = Collections.min(values);
        double max = Collections.max(values);
        return ((max - min) / mean) * 100.0;
    }

    private void saveInchesPerTick(double value) {
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            ReadWriteFile.writeFile(file, String.format(Locale.US, "%.8f", value));
        } catch (Exception ignored) {}
    }
}
