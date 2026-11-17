package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * TurretFieldConfig
 *
 * Master configuration class containing all measured hardware dimensions, pod offsets,
 * start profiles, target heights, and companion CONFIRMED flags.
 */
@Config
public class TurretFieldConfig {

    // =========================================================================
    // 1. PINPOINT ENCODER RESOLUTION & POD CONFIGURATION
    // =========================================================================

    // Inches of pod travel per raw encoder tick
    // TODO(user): MEASURE via InchesPerTickCalibrationOpMode
    public static double INCHES_PER_TICK = 0.00197945; // ~505.2 ticks/inch (19.89 ticks/mm)
    public static boolean INCHES_PER_TICK_CONFIRMED = false;

    // Toggle custom INCHES_PER_TICK resolution vs goBILDA OdometryPods enum
    public static boolean USE_CUSTOM_INCHES_PER_TICK = false;

    // Pinpoint Pod Mounting Offsets from tracking point (in MM)
    // TODO(user): MEASURE - Distance from tracking center to X pod and Y pod
    public static double PINPOINT_X_OFFSET_MM = -84.0;
    public static double PINPOINT_Y_OFFSET_MM = -168.0;
    public static boolean PINPOINT_OFFSETS_CONFIRMED = false;

    // Encoder Count Directions
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_X_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // =========================================================================
    // 2. START PROFILES (ROBOT PLACEMENT & BASKET CENTER DELTAS)
    // =========================================================================

    public static class StartProfile {
        public String name;
        public String description;
        public double startHeadingDeg;
        public double basketDxInches; // Forward (+) or Backward (-) distance from start to basket center
        public double basketDyInches; // Left (+) or Right (-) distance from start to basket center
        public boolean confirmed;

        public StartProfile(String name, String description, double startHeadingDeg, double basketDxInches, double basketDyInches, boolean confirmed) {
            this.name = name;
            this.description = description;
            this.startHeadingDeg = startHeadingDeg;
            this.basketDxInches = basketDxInches;
            this.basketDyInches = basketDyInches;
            this.confirmed = confirmed;
        }
    }

    // List of configurable start profiles
    public static final List<StartProfile> START_PROFILES = new ArrayList<>(Arrays.asList(
            new StartProfile(
                    "Red High Basket Start",
                    "Flush against back tile wall, left side alignment mark",
                    0.0,
                    48.0, // TODO(user): MEASURE basketDxInches (inches from start to basket center forward)
                    48.0, // TODO(user): MEASURE basketDyInches (inches from start to basket center left)
                    false
            ),
            new StartProfile(
                    "Blue High Basket Start",
                    "Flush against back tile wall, right side alignment mark",
                    0.0,
                    48.0, // TODO(user): MEASURE basketDxInches
                    -48.0,// TODO(user): MEASURE basketDyInches
                    false
            )
    ));

    // Index of currently selected profile
    public static int SELECTED_PROFILE_INDEX = 0;

    public static StartProfile getSelectedProfile() {
        if (SELECTED_PROFILE_INDEX < 0 || SELECTED_PROFILE_INDEX >= START_PROFILES.size()) {
            SELECTED_PROFILE_INDEX = 0;
        }
        return START_PROFILES.get(SELECTED_PROFILE_INDEX);
    }

    // =========================================================================
    // 3. TARGET & BALLISTIC HEIGHTS (INCHES)
    // =========================================================================

    // Target Basket Rim Height above tiles (Inches)
    // TODO(user): MEASURE (Into the Deep High Basket rim is 43.0 inches)
    public static double RIM_HEIGHT_INCHES = 43.0;
    public static boolean RIM_HEIGHT_CONFIRMED = false;

    // Clearance above basket rim to aim projectile center (Inches)
    // TODO(user): MEASURE / TUNE
    public static double TARGET_CLEARANCE_INCHES = 3.5;
    public static boolean TARGET_CLEARANCE_CONFIRMED = false;

    /** Derived Target Height = Rim Height + Clearance */
    public static double getTargetHeightInches() {
        return RIM_HEIGHT_INCHES + TARGET_CLEARANCE_INCHES;
    }

    // Height of ball center as it leaves the turret launch ramp (Inches)
    // TODO(user): MEASURE
    public static double TURRET_EXIT_HEIGHT_INCHES = 12.0;
    public static boolean TURRET_EXIT_HEIGHT_CONFIRMED = false;

    // Horizontal forward offset of ball exit point ahead of turret pivot axis (Inches)
    // TODO(user): MEASURE
    public static double EXIT_FORWARD_OFFSET_INCHES = 0.0;
    public static boolean EXIT_FORWARD_OFFSET_CONFIRMED = false;

    // Turret pivot axis mounting offsets from Pinpoint tracking point (Inches)
    // TODO(user): MEASURE
    public static double TURRET_OFFSET_X_INCHES = 0.0;
    public static double TURRET_OFFSET_Y_INCHES = 0.0;
    public static boolean TURRET_OFFSETS_CONFIRMED = false;

    // Flywheel wheel radius (inches from motor shaft center to wheel edge)
    // TODO(user): MEASURE
    public static double FLYWHEEL_RADIUS_INCHES = 2.0;
    public static boolean FLYWHEEL_RADIUS_CONFIRMED = false;

    // Fixed launcher ramp tilt angle relative to ground (degrees)
    // TODO(user): MEASURE
    public static double LAUNCH_ANGLE_DEGREES = 50.0;
    public static boolean LAUNCH_ANGLE_CONFIRMED = false;

    // =========================================================================
    // 4. TURRET ANGLE LIMITS & SERVO RANGE
    // =========================================================================

    // Turret total degrees sweep for 0.0 to 1.0 servo position
    // TODO(user): MEASURE via Two-Point Servo Calibration
    public static double SERVO_TOTAL_RANGE_DEGREES = 180.0;
    public static boolean SERVO_RANGE_CONFIRMED = false;

    // Turret hard-stop and cable wrapping angle limits (degrees)
    // TODO(user): MEASURE
    public static double MIN_TURRET_ANGLE_DEG = -135.0;
    public static double MAX_TURRET_ANGLE_DEG = 135.0;
    public static boolean TURRET_LIMITS_CONFIRMED = false;

    // =========================================================================
    // UNCONFIRMED CALIBRATION DIAGNOSTICS
    // =========================================================================

    /**
     * Returns a list of names for all calibration constants that are unconfirmed.
     */
    public static List<String> getUnconfirmedCalibrations() {
        List<String> unconfirmed = new ArrayList<>();
        if (!INCHES_PER_TICK_CONFIRMED) unconfirmed.add("INCHES_PER_TICK");
        if (!PINPOINT_OFFSETS_CONFIRMED) unconfirmed.add("PINPOINT_OFFSETS (X/Y mm)");
        if (!getSelectedProfile().confirmed) unconfirmed.add("START_PROFILE (" + getSelectedProfile().name + ")");
        if (!RIM_HEIGHT_CONFIRMED) unconfirmed.add("RIM_HEIGHT_INCHES");
        if (!TARGET_CLEARANCE_CONFIRMED) unconfirmed.add("TARGET_CLEARANCE_INCHES");
        if (!TURRET_EXIT_HEIGHT_CONFIRMED) unconfirmed.add("TURRET_EXIT_HEIGHT_INCHES");
        if (!EXIT_FORWARD_OFFSET_CONFIRMED) unconfirmed.add("EXIT_FORWARD_OFFSET_INCHES");
        if (!TURRET_OFFSETS_CONFIRMED) unconfirmed.add("TURRET_OFFSETS_INCHES (X/Y)");
        if (!FLYWHEEL_RADIUS_CONFIRMED) unconfirmed.add("FLYWHEEL_RADIUS_INCHES");
        if (!LAUNCH_ANGLE_CONFIRMED) unconfirmed.add("LAUNCH_ANGLE_DEGREES");
        if (!SERVO_RANGE_CONFIRMED) unconfirmed.add("SERVO_TOTAL_RANGE_DEGREES");
        if (!TURRET_LIMITS_CONFIRMED) unconfirmed.add("TURRET_LIMITS_DEG (Min/Max)");
        return unconfirmed;
    }
}
