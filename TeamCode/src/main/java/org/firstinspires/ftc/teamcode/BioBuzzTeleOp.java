package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/**
 * BioBuzzTeleOp
 *
 * Production FTC Competition TeleOp featuring Field-Centric / Robot-Centric Mecanum Drive,
 * integrated with AutoTurretShooter subsystem for SOTM auto-aiming, dynamic sample switching,
 * basket mode cycling, FTC Dashboard live telemetry & graphing, and readiness-gated auto-firing / rumble feedback.
 */
@TeleOp(name = "BioBuzz Competition TeleOp", group = "Competition")
public class BioBuzzTeleOp extends LinearOpMode {

    // Subsystem Instance
    private final AutoTurretShooter turretShooter = new AutoTurretShooter();

    // Mecanum Drive Motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // Competition State Variables
    private AutoTurretShooter.SampleType currentSample = AutoTurretShooter.SampleType.SMALL;
    private AutoTurretShooter.TargetBasketMode basketMode = AutoTurretShooter.TargetBasketMode.AUTO_NEAREST;
    private boolean enableSOTM = true;

    // Edge Detection Memory
    private boolean lastA = false;
    private boolean lastY = false;
    private boolean lastX = false;
    private boolean lastBack = false;

    @Override
    public void runOpMode() {
        // Wrap Telemetry with FTC Dashboard MultipleTelemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Mecanum Drive Motors
        try {
            leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
            rightFront = hardwareMap.get(DcMotor.class, "rightFront");
            leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
            rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.REVERSE);

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            telemetry.addData("WARNING", "Drive motors not configured in hardwareMap!");
        }

        // Initialize AutoTurretShooter Subsystem with Pre-Configured Start Pose (138.0, 30.2, 180.0 deg)
        Pose2D startPose = PoseStorage.hasSavedPose ? PoseStorage.currentPose :
                new Pose2D(DistanceUnit.INCH, AutoTurretShooter.INIT_ROBOT_X, AutoTurretShooter.INIT_ROBOT_Y, AngleUnit.DEGREES, AutoTurretShooter.INIT_ROBOT_HEADING_DEG);

        turretShooter.init(hardwareMap, startPose, AutoTurretShooter.AllianceColor.RED);

        telemetry.addLine("=== BIOBUZZ TELEOP INITIALIZED ===");
        telemetry.addData("Start Pose", String.format(Locale.US, " (%.1f, %.1f in, %.0f°)", startPose.getX(DistanceUnit.INCH), startPose.getY(DistanceUnit.INCH), startPose.getHeading(AngleUnit.DEGREES)));
        telemetry.addData("Health Status", turretShooter.isHealthy() ? "HEALTHY" : "CRITICAL UNHEALTHY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // -----------------------------------------------------------------
            // 1. GAMEPAD INPUTS & MODE TOGGLES
            // -----------------------------------------------------------------

            // Gamepad 1 [A]: Toggle Sample Type (SMALL vs LARGE)
            if (gamepad1.a && !lastA) {
                currentSample = (currentSample == AutoTurretShooter.SampleType.SMALL) ?
                        AutoTurretShooter.SampleType.LARGE : AutoTurretShooter.SampleType.SMALL;
            }
            lastA = gamepad1.a;

            // Gamepad 1 [Y]: Cycle Target Basket Mode
            if (gamepad1.y && !lastY) {
                int nextIndex = (basketMode.ordinal() + 1) % AutoTurretShooter.TargetBasketMode.values().length;
                basketMode = AutoTurretShooter.TargetBasketMode.values()[nextIndex];
            }
            lastY = gamepad1.y;

            // Gamepad 1 [X]: Toggle Shoot-on-the-Move (SOTM) Compensation
            if (gamepad1.x && !lastX) {
                enableSOTM = !enableSOTM;
            }
            lastX = gamepad1.x;

            // Gamepad 1 [BACK]: Re-zero position to Initial Pose (138.0, 30.2, 180 deg)
            if (gamepad1.back && !lastBack) {
                PoseStorage.currentPose = new Pose2D(DistanceUnit.INCH, AutoTurretShooter.INIT_ROBOT_X, AutoTurretShooter.INIT_ROBOT_Y, AngleUnit.DEGREES, AutoTurretShooter.INIT_ROBOT_HEADING_DEG);
            }
            lastBack = gamepad1.back;

            // Trigger / Bumper: Spin Up Shooter
            boolean spinUpShooter = gamepad1.right_bumper || gamepad1.right_trigger > 0.1;

            // -----------------------------------------------------------------
            // 2. SUBSYSTEM AUTO-AIM UPDATE
            // -----------------------------------------------------------------
            AutoTurretShooter.AimResult aim = turretShooter.updateAutoAim(currentSample, basketMode, spinUpShooter, enableSOTM);

            // -----------------------------------------------------------------
            // 3. AUTO-GATED FEEDING & RUMBLE FEEDBACK
            // -----------------------------------------------------------------
            if (spinUpShooter && turretShooter.isReadyToShoot()) {
                turretShooter.actuateFeedServo(true);
                gamepad1.rumble(100); // 100ms haptic feedback blip to driver
            } else {
                turretShooter.actuateFeedServo(false);
            }

            // -----------------------------------------------------------------
            // 4. MECANUM DRIVE (Robot-Centric / Field-Centric)
            // -----------------------------------------------------------------
            if (leftFront != null && rightFront != null && leftBack != null && rightBack != null) {
                double drive  = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn   = gamepad1.right_stick_x;

                double lfPower = drive + strafe + turn;
                double rfPower = drive - strafe - turn;
                double lbPower = drive - strafe + turn;
                double rbPower = drive + strafe - turn;

                // Normalize powers
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

            // -----------------------------------------------------------------
            // 5. DRIVER HUD TELEMETRY
            // -----------------------------------------------------------------
            telemetry.addLine("=== BIOBUZZ COMPETITION HUD ===");
            telemetry.addData("Sample Type", currentSample + " (Press [A] to toggle)");
            telemetry.addData("Basket Mode", basketMode + " (Press [Y] to cycle)");
            telemetry.addData("SOTM Status", enableSOTM ? "ENABLED (COMPENSATING)" : "DISABLED (STATIC)");
            telemetry.addData("CAN FIRE?", turretShooter.isReadyToShoot() ? "READY TO FIRE! (AUTO-FEEDING)" : "BLOCKED (" + aim.blockedReason + ")");
            telemetry.addLine("----------------------------------------");

            // Subsystem Detailed Telemetry (also draws to FTC Dashboard)
            turretShooter.sendTelemetry(telemetry);
        }
    }
}
