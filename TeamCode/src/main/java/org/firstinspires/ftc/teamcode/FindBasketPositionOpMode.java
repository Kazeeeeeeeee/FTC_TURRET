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
 *
 * HOW TO USE THIS FILE:
 * 1. Place the robot at your fixed starting position on the field.
 * 2. Start this OpMode. The starting position will be automatically set to (x = 0, y = 0).
 * 3. Manually push or drive the robot directly under the center of the target basket.
 *    - Forward movement increases X (+X)
 *    - Leftward movement increases Y (+Y)
 * 4. Press GAMEPAD 1 [A] (or Cross on PlayStation controller) to snapshot/lock the basket position.
 * 5. Note down the recorded Basket X and Basket Y values displayed on the Driver Station.
 * 6. Copy those coordinates into your startup file (TurretAutoAimTeleOp.java or TurretAimCalculator.java)!
 */
@TeleOp(name = "Find Basket 2D Position Calibration", group = "Calibration")
public class FindBasketPositionOpMode extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    // Recorded Basket Position variables
    private double markedBasketXInches = 0.0;
    private double markedBasketYInches = 0.0;
    private double markedBasketHeadingDeg = 0.0;
    private boolean positionMarked = false;

    @Override
    public void runOpMode() {
        // Initialize the goBILDA Pinpoint Odometry Computer
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Standard goBILDA Pinpoint offset configuration (Adjust offsets if needed for your robot hardware setup)
        // X pod offset: distance sideways from tracking point (left = positive, right = negative)
        // Y pod offset: distance forwards from tracking point (forward = positive, backward = negative)
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);

        telemetry.addData("Status", "Calibrating Pinpoint IMU...");
        telemetry.update();

        // Reset position and recalibrate IMU at starting point
        pinpoint.resetPosAndIMU();

        // Explicitly set starting position to default (x = 0, y = 0, heading = 0)
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        telemetry.addLine("=== BASKET 2D POSITION FINDER READY ===");
        telemetry.addLine("Robot position initialized to (X=0, Y=0).");
        telemetry.addLine("Press START on Driver Station to begin calibration.");
        telemetry.update();

        waitForStart();

        boolean lastAState = false;
        boolean lastBState = false;

        while (opModeIsActive()) {
            // Update Odometry Computer readings
            pinpoint.update();
            Pose2D currentPose = pinpoint.getPosition();

            double currentXInches = currentPose.getX(DistanceUnit.INCH);
            double currentYInches = currentPose.getY(DistanceUnit.INCH);
            double currentHeadingDeg = currentPose.getHeading(AngleUnit.DEGREES);

            // GAMEPAD 1 [A]: Mark down current position as Basket position
            boolean currentAState = gamepad1.a;
            if (currentAState && !lastAState) {
                markedBasketXInches = currentXInches;
                markedBasketYInches = currentYInches;
                markedBasketHeadingDeg = currentHeadingDeg;
                positionMarked = true;
            }
            lastAState = currentAState;

            // GAMEPAD 1 [B]: Reset Odometry zero position to current robot position
            boolean currentBState = gamepad1.b;
            if (currentBState && !lastBState) {
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
                positionMarked = false;
            }
            lastBState = currentBState;

            // Telemetry Output
            telemetry.addLine("========================================");
            telemetry.addLine("       BASKET POSITION CALIBRATOR       ");
            telemetry.addLine("========================================");
            telemetry.addLine("INSTRUCTIONS:");
            telemetry.addLine("1. Move robot directly right under the Basket.");
            telemetry.addLine("2. Press [A] on Gamepad 1 to LOCK the basket position.");
            telemetry.addLine("3. Press [B] to RESET starting position back to (0, 0).");
            telemetry.addLine("----------------------------------------");

            // Live Odometry Position
            telemetry.addData("LIVE Robot X (Forward +)", String.format(Locale.US, "%.2f inches", currentXInches));
            telemetry.addData("LIVE Robot Y (Left +)",    String.format(Locale.US, "%.2f inches", currentYInches));
            telemetry.addData("LIVE Heading",              String.format(Locale.US, "%.1f deg",    currentHeadingDeg));
            telemetry.addLine("----------------------------------------");

            // Marked Basket Position
            if (positionMarked) {
                telemetry.addLine(">>> BASKET POSITION MARKED & LOCKED! <<<");
                telemetry.addData("--> MARKED Basket X", String.format(Locale.US, "%.2f inches", markedBasketXInches));
                telemetry.addData("--> MARKED Basket Y", String.format(Locale.US, "%.2f inches", markedBasketYInches));
                telemetry.addData("--> MARKED Heading",  String.format(Locale.US, "%.1f deg",    markedBasketHeadingDeg));
                telemetry.addLine("");
                telemetry.addLine("COPY THESE VARIABLES INTO YOUR STARTUP FILE:");
                telemetry.addLine(String.format(Locale.US, "aimCalculator.basketXInches = %.2f;", markedBasketXInches));
                telemetry.addLine(String.format(Locale.US, "aimCalculator.basketYInches = %.2f;", markedBasketYInches));
            } else {
                telemetry.addLine("Press [A] to mark down the Basket position when right under it.");
            }

            telemetry.addLine("========================================");
            telemetry.update();

            // Short sleep to lighten CPU loop load
            sleep(20);
        }
    }
}
