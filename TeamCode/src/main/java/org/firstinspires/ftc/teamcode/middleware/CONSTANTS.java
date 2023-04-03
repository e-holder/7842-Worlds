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

    public enum VeraPipelineType {
        SIGNAL,
        FIND_POLE
    }

    public enum CornerType {
        W, // Wall
        G, // Ground Junction
        L, // Low Junction
        M, // Medium Junction
        H  // High Junction
    }

    public enum Rotation {
        AUTO,
        FORCE_CLOCKWISE,
        FORCE_COUNTERCLOCKWISE
    }

    public enum CompassHeading {
        UNINITIALIZED(45),  // <---- Cause behavior that will be obviously wrong.
        WEST(-90),
        NORTH(0),
        EAST(90),
        SOUTH(180);

        private final int m_degrees;
        private CompassHeading(int degrees) { this.m_degrees = degrees; }
        public double toDegrees() { return (double)m_degrees; }
    }

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
