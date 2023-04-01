package org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks;

import android.service.quicksettings.Tile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.middleware.Vera;
import org.firstinspires.ftc.teamcode.middleware.VisionPipelineSignal;

// Game-specific field locations, constants, and information
public abstract class AutonomousTask implements CONSTANTS {

    // Provide access to commonly used objects in AutonomousTask child classes.
    // NOTE: These public and protected static variables are not a very "correct" way to do things.
    //       It violates some basic software design principles, but was quick to put in place and
    //       makes things very easy.
    public static Vera vera = null;  // Needed for access to CSV logging & middleware subsystems.
    public static Alliance alliance = Alliance.UNDEFINED;
    public static Telemetry telemetry = null;
    public static FieldSide fieldSide = FieldSide.UNDEFINED;
    protected static VisionPipelineSignal.Signal signal = VisionPipelineSignal.Signal.UNKNOWN;

    // All child classes (tasks which extend AutonomousTask) will have to provide an update method.
    public abstract TaskStatus update(double runTime_s);
}
