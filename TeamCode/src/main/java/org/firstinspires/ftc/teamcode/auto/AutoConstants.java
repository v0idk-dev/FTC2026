package org.firstinspires.ftc.teamcode.auto;

public final class AutoConstants {

    public static final class Robot {
        public static final double START_WIDTH = 18; // Units are all inches.
        public static final double START_LENGTH = 18;
        public static final double STARTING_HEIGHT = 18;
        public static final double START_DISTANCE_FROM_WALL = START_LENGTH / 2;
    }

    public class Field {
        public static final double WIDTH = 144;
        public static final double HEIGHT = 144;
        public static final double HALF_WIDTH = WIDTH / 2;
        public static final double HALF_HEIGHT = HEIGHT / 2;
    }
    // =========================================================================================
    // #region CONFIG NAMES
    // =========================================================================================

    // Pedropathing uses a 144 x 144 field, where the bottom-left corner is (0,0). Think x-y graphs.

    public class LoadingZone {
        public static final double LENGTH = 20.692;
        public static final double WIDTH = 10;

        public static final double X = 0;  // One end of the field
        public static final double Y = Field.HEIGHT / 12 * 3;  // Middle of the Loading Zone

    }

    // =========================================================================================
    // #region DERIVED CONSTANTS -- DO NOT ADD VALUES
    // =========================================================================================

}
