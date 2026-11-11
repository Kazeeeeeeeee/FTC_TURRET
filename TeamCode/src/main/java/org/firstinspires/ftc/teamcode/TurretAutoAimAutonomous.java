package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/**
 * TurretAutoAimAutonomous
 *
 * Example Autonomous OpMode illustrating how to aim and shoot at the basket
 * programmatically using goBILDA Pinpoint odometry and TurretAimCalculator.
 */
@Autonomous(name = "Turret Auto Aim Autonomous Sample", group = "Autonomous")
public class TurretAutoAimAutonomous extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;
    private Servo turretServo;
    private DcMotorEx flywheelMotor;
    private Servo feedServo;

    private final TurretAimCalculator aimCalculator = new TurretAimCalculator();

    @Override
    public void runOpMode() {
        // Setup Aim Calculator parameters (same as in TeleOp)
        aimCalculator.flywheelRadiusInches = 2.0;
        aimCalculator.turretOffsetXInches = 0.0;
        aimCalculator.turretOffsetYInches = 0.0;
        aimCalculator.turretHeightInches = 12.0;

        aimCalculator.basketXInches = 48.0;  // Set from FindBasketPositionOpMode calibration
        aimCalculator.basketYInches = 48.0;
        aimCalculator.basketHeightInches = 43.0;
        aimCalculator.launchAngleDegrees = 50.0;
        aimCalculator.flywheelEfficiency = 0.65;
        aimCalculator.maxMotorRPM = 6000.0;

        // Initialize Hardware
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        turretServo = hardwareMap.get(Servo.class, "turretServo");
        // Initialize shooter turret facing DIRECTLY FORWARD relative to robot
        turretServo.setPosition(aimCalculator.getForwardServoPosition());

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        try {
            feedServo = hardwareMap.get(Servo.class, "feedServo");
            feedServo.setPosition(0.0);
        } catch (Exception e) {
            feedServo = null;
        }

        telemetry.addLine("Auto-Aim Autonomous Ready. Turret initialized facing DIRECTLY FORWARD.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Update Odometry position
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            // Calculate auto-aim solution
            TurretAimCalculator.AimResult aim = aimCalculator.calculateAim(pose);

            telemetry.addData("Aim Status", aim.statusMessage);
            telemetry.addData("Target Distance", String.format(Locale.US, "%.2f in", aim.horizontalDistanceInches));
            telemetry.addData("Target Servo Pos", String.format(Locale.US, "%.3f", aim.servoPosition));
            telemetry.addData("Required Motor Power", String.format(Locale.US, "%.1f%%", aim.motorPower * 100.0));
            telemetry.update();

            if (aim.targetInReach) {
                // Step 1: Aim turret servo
                turretServo.setPosition(aim.servoPosition);

                // Step 2: Spin up flywheel motor to required speed
                flywheelMotor.setPower(aim.motorPower);

                // Wait 1 second for flywheel to reach target RPM
                sleep(1000);

                // Step 3: Trigger shoot if feed servo is available
                if (feedServo != null) {
                    feedServo.setPosition(0.5); // Push sample into shooter
                    sleep(500);
                    feedServo.setPosition(0.0);
                }

                // Wait for shot completion and turn off flywheel motor
                sleep(500);
                flywheelMotor.setPower(0.0);
            }
        }
    }
}
