package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

/**
 * BasketAimCheckOpMode
 *
 * Driver-Station OpMode to verify and fine-tune turret aiming before shooting (Flywheel & Feed Disabled, Drive Enabled).
 *
 * CONTROLS:
 * - Gamepad 1 Sticks: Mecanum Drive
 * - Gamepad 2 D-Pad Up/Down: Nudge basket Dx (+/- 0.25" or 1.0")
 * - Gamepad 2 D-Pad Left/Right: Nudge basket Dy (+/- 0.25" or 1.0")
 * - Gamepad 2 Bumpers: Toggle step size (0.25 in vs 1.0 in)
 * - Gamepad 2 [X]: Save nudged Basket position to settings file & print StartProfile snippet
 */
@TeleOp(name = "Basket Aim Check (Drive Only)", group = "Diagnostics")
public class BasketAimCheckOpMode extends OpMode {

    private final TurretSubsystem turret = new TurretSubsystem();

    // Drive Motors
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private boolean hasDriveMotors = false;

    // Nudged Basket Position
    private double nudgedDxInches = 48.0;
    private double nudgedDyInches = 48.0;
    private double stepSizeInches = 0.25;

    // Measured Tape Baseline Values
    private double tapeDxInches = 48.0;
    private double tapeDyInches = 48.0;

    // Edge Detection
    private boolean lastDUp = false;
    private boolean lastDDown = false;
    private boolean lastDLeft = false;
    private boolean lastDRight = false;
    private boolean lastBumper = false;
    private boolean lastX = false;

    public static final String FILE_NAME = "basket_position.txt";

    @Override
    public void init() {
        // Flywheel disabled for safety during aim check
        turret.flywheelEnabled = false;
        turret.init(hardwareMap);

        TurretFieldConfig.StartProfile profile = TurretFieldConfig.getSelectedProfile();
        nudgedDxInches = profile.basketDxInches;
        nudgedDyInches = profile.basketDyInches;
        tapeDxInches = profile.basketDxInches;
        tapeDyInches = profile.basketDyInches;

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

        telemetry.addLine("=== BASKET AIM CHECK READY ===");
        telemetry.addLine("Drive around and observe turret aiming.");
        telemetry.addLine("Gamepad 2 D-Pad nudges basket Dx/Dy.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Gamepad 2 Bumper toggles step size between 0.25 in and 1.0 in
        boolean currentBumper = gamepad2.left_bumper || gamepad2.right_bumper;
        if (currentBumper && !lastBumper) {
            stepSizeInches = (stepSizeInches == 0.25) ? 1.0 : 0.25;
        }
        lastBumper = currentBumper;

        // Gamepad 2 D-Pad Up/Down: Nudge Dx (Forward/Backward)
        if (gamepad2.dpad_up && !lastDUp) {
            nudgedDxInches += stepSizeInches;
        }
        lastDUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastDDown) {
            nudgedDxInches -= stepSizeInches;
        }
        lastDDown = gamepad2.dpad_down;

        // Gamepad 2 D-Pad Left/Right: Nudge Dy (Left/Right)
        if (gamepad2.dpad_left && !lastDLeft) {
            nudgedDyInches += stepSizeInches;
        }
        lastDLeft = gamepad2.dpad_left;

        if (gamepad2.dpad_right && !lastDRight) {
            nudgedDyInches -= stepSizeInches;
        }
        lastDRight = gamepad2.dpad_right;

        // Apply nudged basket position
        turret.aimCalculator.basketXInches = nudgedDxInches;
        turret.aimCalculator.basketYInches = nudgedDyInches;

        // Update Auto-Aim (Flywheel disabled)
        TurretAimCalculator.AimResult aim = turret.updateAutoAim(false);

        // Gamepad 2 [X]: Save to file
        if (gamepad2.x && !lastX) {
            saveBasketPosition(nudgedDxInches, nudgedDyInches);
        }
        lastX = gamepad2.x;

        // Mecanum Drive (Gamepad 1)
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

        // Telemetry Display
        Pose2D pose = turret.getCurrentPose();
        double deltaDx = nudgedDxInches - tapeDxInches;
        double deltaDy = nudgedDyInches - tapeDyInches;

        telemetry.addLine("========================================");
        telemetry.addLine("       BASKET AIM CHECK DIAGNOSTIC      ");
        telemetry.addLine("========================================");
        telemetry.addData("Nudge Step Size", String.format(Locale.US, "%.2f in (Press Bumper to toggle)", stepSizeInches));
        telemetry.addData("Nudged Basket Dx, Dy", String.format(Locale.US, "%.2f, %.2f in", nudgedDxInches, nudgedDyInches));
        telemetry.addData("Tape vs Nudge Diff", String.format(Locale.US, "ΔDx=%.2f in, ΔDy=%.2f in", deltaDx, deltaDy));
        telemetry.addLine("----------------------------------------");
        telemetry.addData("Robot Pose (X, Y)", String.format(Locale.US, "%.2f, %.2f in", pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH)));
        telemetry.addData("Heading (Yaw)", String.format(Locale.US, "%.1f deg", pose.getHeading(AngleUnit.DEGREES)));
        telemetry.addData("Distance to Basket", String.format(Locale.US, "%.2f in", aim.horizontalDistanceInches));
        telemetry.addData("Field Angle to Basket", String.format(Locale.US, "%.1f deg", aim.turretFieldAngleDegrees));
        telemetry.addData("Turret Relative Angle", String.format(Locale.US, "%.1f deg", aim.turretRelativeAngleDegrees));
        telemetry.addData("Turret Servo Position", String.format(Locale.US, "%.3f", aim.servoPosition));
        telemetry.addData("Turret At Limit?", aim.turretAtLimit ? "YES" : "NO");
        telemetry.addData("Shot Valid?", aim.shotValid ? "VALID" : "INVALID (" + aim.statusMessage + ")");
        telemetry.addData("Required Flywheel RPM", String.format(Locale.US, "%.0f RPM", aim.requiredFlywheelRPM));
        telemetry.addLine("----------------------------------------");
        telemetry.addLine("PASTE INTO StartProfile:");
        telemetry.addLine(String.format(Locale.US, "basketDxInches = %.2f, basketDyInches = %.2f", nudgedDxInches, nudgedDyInches));
        telemetry.addLine("Press Gamepad 2 [X] to SAVE to settings file!");
        telemetry.addLine("========================================");
        telemetry.update();
    }

    private void saveBasketPosition(double dx, double dy) {
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            ReadWriteFile.writeFile(file, String.format(Locale.US, "%.2f,%.2f", dx, dy));
        } catch (Exception ignored) {}
    }
}
