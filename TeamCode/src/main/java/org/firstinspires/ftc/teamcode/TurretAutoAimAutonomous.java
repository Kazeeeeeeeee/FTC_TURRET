package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/**
 * TurretAutoAimAutonomous
 *
 * Example Autonomous OpMode illustrating how to aim and shoot at the basket
 * using TurretSubsystem during autonomous match play, saving end pose to PoseStorage.
 */
@Autonomous(name = "Turret Auto Aim Autonomous Sample", group = "Autonomous")
public class TurretAutoAimAutonomous extends LinearOpMode {

    private final TurretSubsystem turret = new TurretSubsystem();

    @Override
    public void runOpMode() {
        turret.alliance = Alliance.RED;
        turret.setStartPose(0.0, 0.0, 0.0);
        turret.init(hardwareMap);

        telemetry.addLine("Auto-Aim Autonomous Ready.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Update auto-aim logic
            TurretAimCalculator.AimResult aim = turret.updateAutoAim(false);

            telemetry.addData("Aim Status", aim.statusMessage);
            telemetry.addData("Target Distance", String.format(Locale.US, "%.2f in", aim.horizontalDistanceInches));
            telemetry.addData("Can Fire?", turret.canFire ? "YES" : "NO (" + turret.blockedReason + ")");
            telemetry.update();

            if (turret.canFire) {
                // Wait for flywheel to reach target speed
                sleep(1000);

                // Actuate feed state machine trigger
                long startMs = System.currentTimeMillis();
                while (opModeIsActive() && System.currentTimeMillis() - startMs < 1000) {
                    turret.updateAutoAim(true);
                    sleep(20);
                }
            }

            // Save final autonomous pose to PoseStorage for TeleOp transition
            PoseStorage.currentPose = turret.getCurrentPose();
            PoseStorage.hasSavedPose = true;
        }
    }
}
