package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * PoseStorage
 *
 * Static holder class to persist robot pose across OpMode transitions (e.g. Autonomous to TeleOp).
 */
public class PoseStorage {
    public static Pose2D currentPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    public static boolean hasSavedPose = false;
}
