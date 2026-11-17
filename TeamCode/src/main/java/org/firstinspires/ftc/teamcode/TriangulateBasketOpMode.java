package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * TriangulateBasketOpMode
 *
 * Calibrates 2D Basket position (X, Y) via least-squares ray intersection from 2 or more spots.
 *
 * HOW TO USE:
 * 1. Move robot to Spot 1. Rotate turret using right stick until aiming visibly at basket. Press [A].
 * 2. Move robot to Spot 2 (different position & angle). Aim turret at basket. Press [A].
 * 3. OpMode computes least-squares intersection (basketX, basketY). Press [B] to save!
 */
@TeleOp(name = "Triangulate Basket Position", group = "Calibration")
public class TriangulateBasketOpMode extends LinearOpMode {

    public static class SightingRay {
        public double posX, posY;
        public double dirX, dirY;
        public double angleRad;

        public SightingRay(double posX, double posY, double angleRad) {
            this.posX = posX;
            this.posY = posY;
            this.angleRad = angleRad;
            this.dirX = Math.cos(angleRad);
            this.dirY = Math.sin(angleRad);
        }
    }

    private final TurretSubsystem turret = new TurretSubsystem();
    private final List<SightingRay> rays = new ArrayList<>();
    private double manualTurretAngleDeg = 0.0;

    private static final String POSITION_FILE_NAME = "basket_position.txt";

    @Override
    public void runOpMode() {
        turret.init(hardwareMap);

        telemetry.addLine("=== TRIANGULATE BASKET OPMODE READY ===");
        telemetry.addLine("Aim turret at basket from 2+ spots and press [A].");
        telemetry.update();

        waitForStart();

        boolean lastA = false;
        boolean lastB = false;
        boolean lastX = false;

        double calculatedBasketX = Double.NaN;
        double calculatedBasketY = Double.NaN;

        while (opModeIsActive()) {
            Pose2D pose = turret.getCurrentPose();

            // Manual turret rotation using Right Stick X
            manualTurretAngleDeg += gamepad1.right_stick_x * 1.5;
            manualTurretAngleDeg = TurretAimCalculator.normalizeAngleDegrees(manualTurretAngleDeg);

            // Command turret servo to manual angle
            double servoPos = turret.aimCalculator.calculateServoPosition(manualTurretAngleDeg);
            // Directly set servo position
            turret.updateAutoAim(false);

            // Field Angle of sighting ray = robot heading + turret relative angle
            double fieldAngleDeg = TurretAimCalculator.normalizeAngleDegrees(pose.getHeading(AngleUnit.DEGREES) + manualTurretAngleDeg);
            double fieldAngleRad = Math.toRadians(fieldAngleDeg);

            // Press [A] to add sighting ray
            if (gamepad1.a && !lastA) {
                boolean validRay = true;
                for (SightingRay r : rays) {
                    double deltaAngle = Math.abs(TurretAimCalculator.normalizeAngleDegrees(fieldAngleDeg - Math.toDegrees(r.angleRad)));
                    if (Math.abs(Math.sin(Math.toRadians(deltaAngle))) < 0.15) {
                        validRay = false; // Reject near-parallel rays
                        break;
                    }
                }
                if (validRay) {
                    rays.add(new SightingRay(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), fieldAngleRad));
                }
            }
            lastA = gamepad1.a;

            // Press [X] to clear rays
            if (gamepad1.x && !lastX) {
                rays.clear();
                calculatedBasketX = Double.NaN;
                calculatedBasketY = Double.NaN;
            }
            lastX = gamepad1.x;

            // Compute Least-Squares Intersection if 2+ rays exist
            if (rays.size() >= 2) {
                double a11 = 0, a12 = 0, a22 = 0;
                double b1 = 0, b2 = 0;

                for (SightingRay r : rays) {
                    double ux = r.dirX;
                    double uy = r.dirY;

                    double m11 = uy * uy;
                    double m12 = -ux * uy;
                    double m22 = ux * ux;

                    a11 += m11;
                    a12 += m12;
                    a22 += m22;

                    b1 += m11 * r.posX + m12 * r.posY;
                    b2 += m12 * r.posX + m22 * r.posY;
                }

                double det = a11 * a22 - a12 * a12;
                if (Math.abs(det) > 1e-4) {
                    calculatedBasketX = (a22 * b1 - a12 * b2) / det;
                    calculatedBasketY = (-a12 * b1 + a11 * b2) / det;
                }
            }

            // Press [B] to save calculated basket position
            if (gamepad1.b && !lastB && !Double.isNaN(calculatedBasketX)) {
                saveBasketPosition(calculatedBasketX, calculatedBasketY);
            }
            lastB = gamepad1.b;

            // Telemetry Output
            telemetry.addLine("========================================");
            telemetry.addLine("      BASKET TRIANGULATION CALIBRATION  ");
            telemetry.addLine("========================================");
            telemetry.addData("Recorded Sighting Rays", rays.size());
            telemetry.addData("Manual Turret Angle", String.format(Locale.US, "%.1f deg", manualTurretAngleDeg));
            telemetry.addData("Field Sighting Angle", String.format(Locale.US, "%.1f deg", fieldAngleDeg));
            telemetry.addLine("----------------------------------------");

            if (!Double.isNaN(calculatedBasketX)) {
                telemetry.addLine(">>> TRIANGULATION SUCCESSFUL <<<");
                telemetry.addData("Calculated Basket X", String.format(Locale.US, "%.2f in", calculatedBasketX));
                telemetry.addData("Calculated Basket Y", String.format(Locale.US, "%.2f in", calculatedBasketY));
                telemetry.addLine("Press [B] to SAVE to settings file!");
            } else {
                telemetry.addLine("Aim turret at basket from 2 different spots and press [A]");
            }

            telemetry.addLine("========================================");
            telemetry.update();

            sleep(20);
        }
    }

    private void saveBasketPosition(double x, double y) {
        try {
            File file = AppUtil.getInstance().getSettingsFile(POSITION_FILE_NAME);
            ReadWriteFile.writeFile(file, String.format(Locale.US, "%.2f,%.2f", x, y));
        } catch (Exception ignored) {}
    }
}
