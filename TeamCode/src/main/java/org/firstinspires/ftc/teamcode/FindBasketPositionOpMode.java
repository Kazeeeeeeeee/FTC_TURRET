package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/**
 * FindBasketPositionOpMode
 *
 * Calibration tool to determine the exact 2D position (X, Y) of the target basket.
 */
@TeleOp(name = "Find Basket 2D Position Calibration", group = "Calibration")
public class FindBasketPositionOpMode extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    private double markedBasketXInches = 0.0;
    private double markedBasketYInches = 0.0;
    private double markedBasketHeadingDeg = 0.0;
    private boolean positionMarked = false;

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

        pinpoint.setEncoderDirections(TurretFieldConfig.PINPOINT_X_DIRECTION, TurretFieldConfig.PINPOINT_Y_DIRECTION);

        telemetry.addData("Status", "Calibrating Pinpoint IMU...");
        telemetry.update();

        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        telemetry.addLine("=== BASKET 2D POSITION FINDER READY ===");
        telemetry.addLine("Robot position initialized to (X=0, Y=0).");
        telemetry.addLine("Press START on Driver Station to begin calibration.");
        telemetry.update();

        waitForStart();

        boolean lastAState = false;
        boolean lastBState = false;

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D currentPose = pinpoint.getPosition();

            double currentXInches = currentPose.getX(DistanceUnit.INCH);
            double currentYInches = currentPose.getY(DistanceUnit.INCH);
            double currentHeadingDeg = currentPose.getHeading(AngleUnit.DEGREES);

            boolean currentAState = gamepad1.a;
            if (currentAState && !lastAState) {
                markedBasketXInches = currentXInches;
                markedBasketYInches = currentYInches;
                markedBasketHeadingDeg = currentHeadingDeg;
                positionMarked = true;
            }
            lastAState = currentAState;

            boolean currentBState = gamepad1.b;
            if (currentBState && !lastBState) {
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
                positionMarked = false;
            }
            lastBState = currentBState;

            telemetry.addLine("========================================");
            telemetry.addLine("       BASKET POSITION CALIBRATOR       ");
            telemetry.addLine("========================================");
            telemetry.addLine("INSTRUCTIONS:");
            telemetry.addLine("1. Move robot directly right under the Basket.");
            telemetry.addLine("2. Press [A] on Gamepad 1 to LOCK the basket position.");
            telemetry.addLine("3. Press [B] to RESET starting position back to (0, 0).");
            telemetry.addLine("----------------------------------------");

            telemetry.addData("LIVE Robot X (Forward +)", String.format(Locale.US, "%.2f inches", currentXInches));
            telemetry.addData("LIVE Robot Y (Left +)",    String.format(Locale.US, "%.2f inches", currentYInches));
            telemetry.addData("LIVE Heading",              String.format(Locale.US, "%.1f deg",    currentHeadingDeg));
            telemetry.addLine("----------------------------------------");

            if (positionMarked) {
                telemetry.addLine(">>> BASKET POSITION MARKED & LOCKED! <<<");
                telemetry.addData("--> MARKED Basket X (dx)", String.format(Locale.US, "%.2f inches", markedBasketXInches));
                telemetry.addData("--> MARKED Basket Y (dy)", String.format(Locale.US, "%.2f inches", markedBasketYInches));
                telemetry.addData("--> MARKED Heading",  String.format(Locale.US, "%.1f deg",    markedBasketHeadingDeg));
                telemetry.addLine("");
                telemetry.addLine("PASTE INTO StartProfile:");
                telemetry.addLine(String.format(Locale.US, "basketDxInches = %.2f, basketDyInches = %.2f;", markedBasketXInches, markedBasketYInches));
            } else {
                telemetry.addLine("Press [A] to mark down the Basket position when right under it.");
            }

            telemetry.addLine("========================================");
            telemetry.update();

            sleep(20);
        }
    }
}
