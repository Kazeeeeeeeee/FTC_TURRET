package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/**
 * PinpointSanityCheckOpMode
 *
 * Sanity check tool to verify goBILDA Pinpoint Odometry Pod directions, offsets, and drift.
 *
 * TEST A (Spin Test):
 * Rotate the robot in place 360 degrees. X and Y should remain near 0, with minimal drift.
 *
 * TEST B (Push Test):
 * 1. Push robot FORWARD 48 inches -> X must increase (+X).
 * 2. Push robot LEFT 48 inches -> Y must increase (+Y).
 */
@TeleOp(name = "Pinpoint Sanity Check", group = "Diagnostics")
public class PinpointSanityCheckOpMode extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(TurretFieldConfig.PINPOINT_X_OFFSET_MM, TurretFieldConfig.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        telemetry.addLine("=== PINPOINT SANITY CHECK READY ===");
        telemetry.addLine("Press START to begin.");
        telemetry.update();

        waitForStart();

        double startX = 0.0;
        double startY = 0.0;

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.DEGREES);

            if (gamepad1.a) {
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
                startX = 0.0;
                startY = 0.0;
            }

            double driftFromStart = Math.hypot(x - startX, y - startY);

            telemetry.addLine("========================================");
            telemetry.addLine("       PINPOINT SANITY CHECK TOOL       ");
            telemetry.addLine("========================================");
            telemetry.addData("Device Status", pinpoint.getDeviceStatus());
            telemetry.addLine("Press [A] on Gamepad 1 to reset (0, 0, 0)");
            telemetry.addLine("----------------------------------------");

            telemetry.addLine("TEST A: SPIN TEST (Rotate robot in place 360 deg)");
            telemetry.addData("Live Heading", String.format(Locale.US, "%.1f deg", heading));
            telemetry.addData("Position Drift from Start", String.format(Locale.US, "%.2f inches", driftFromStart));
            telemetry.addLine("----------------------------------------");

            telemetry.addLine("TEST B: PUSH TEST (Push 48 inches Forward, then Left)");
            telemetry.addData("Robot X (Forward +)", String.format(Locale.US, "%.2f in (Target: +48.0 in)", x));
            telemetry.addData("Robot Y (Left +)",    String.format(Locale.US, "%.2f in (Target: +48.0 in)", y));

            double xProgressPercent = Math.min(100.0, Math.max(0.0, (x / 48.0) * 100.0));
            double yProgressPercent = Math.min(100.0, Math.max(0.0, (y / 48.0) * 100.0));

            telemetry.addData("Forward 48-in Progress", String.format(Locale.US, "%.1f%%", xProgressPercent));
            telemetry.addData("Leftward 48-in Progress", String.format(Locale.US, "%.1f%%", yProgressPercent));
            telemetry.addLine("========================================");
            telemetry.update();

            sleep(20);
        }
    }
}
