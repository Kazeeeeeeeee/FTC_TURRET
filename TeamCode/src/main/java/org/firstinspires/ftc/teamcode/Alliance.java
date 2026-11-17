package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Alliance Enum
 *
 * Defines RED and BLUE alliance basket positions in field coordinates.
 */
public enum Alliance {
    RED,
    BLUE;

    // Field-frame RED Alliance High Basket coordinates (Inches)
    public static final double RED_BASKET_X_INCHES = -56.0;
    public static final double RED_BASKET_Y_INCHES = 56.0;

    /**
     * Gets the Basket 2D position in field frame coordinates for the given alliance.
     *
     * TODO: Field Symmetry Note:
     * The FTC Into the Deep field layout has rotational 180-degree symmetry across field center (0,0).
     * Blue Alliance High Basket is mirrored at (+56, -56) inches.
     * Update these coordinates if your alliance starting quadrant or basket calibration differs.
     */
    public static Pose2D getBasketPosition(Alliance alliance) {
        if (alliance == BLUE) {
            // Rotational 180-degree field symmetry mirror
            return new Pose2D(DistanceUnit.INCH, -RED_BASKET_X_INCHES, -RED_BASKET_Y_INCHES, AngleUnit.DEGREES, 0);
        }
        return new Pose2D(DistanceUnit.INCH, RED_BASKET_X_INCHES, RED_BASKET_Y_INCHES, AngleUnit.DEGREES, 0);
    }
}
