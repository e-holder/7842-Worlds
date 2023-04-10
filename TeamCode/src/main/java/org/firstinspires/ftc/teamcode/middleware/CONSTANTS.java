package org.firstinspires.ftc.teamcode.middleware;

import java.text.DecimalFormat;

public interface CONSTANTS {

    enum Alliance {
        UNDEFINED,
        RED,
        BLUE
    }

    public enum FieldSide {
        UNDEFINED,
        LEFT,
        RIGHT
    }

    public enum TaskStatus {
        RUNNING,
        DONE
    }

    public enum VeraPipelineType {
        SIGNAL,
        FIND_POLE
    }

    public enum PoleType {
        UNINITIALIZED,
        LOW,
        MID,
        HIGH
    }

    // Find Pole Detection flag values for image rows A, B, and C.
    public final int D_ABC = 7;  // 111
    public final int D_AB = 6;   // 110
    public final int D_AC = 5;   // 101
    public final int D_BC = 3;   // 011
    public final int D_A = 4;    // 100
    public final int D_B = 2;    // 010
    public final int D_C = 1;    // 001
    public final int D_NONE = 0;

    // Field constants
    public static final int FIELD_SIZE_TILES = 6;
    public static final int NUM_TILES = FIELD_SIZE_TILES * FIELD_SIZE_TILES;
    public static final double TILE_SIZE_IN = 23.5625;
    public static final double HALF_TILE_SIZE_IN = TILE_SIZE_IN * 0.5;
    // Field dimensions (origin is the center of the field)
    public static final double FIELD_LENGTH_IN = 141.25;
    public static final double FIELD_WIDTH_IN = FIELD_LENGTH_IN;
    public static final double HALF_FIELD_LENGTH_IN = FIELD_LENGTH_IN * 0.5;  // 70.625
    public static final double HALF_FIELD_WIDTH_IN = HALF_FIELD_LENGTH_IN;

    // Logging constants
    public final DecimalFormat df3 = new DecimalFormat("#.###");

}
