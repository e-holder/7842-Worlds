package org.firstinspires.ftc.teamcode.roadrunner.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.middleware.Vera;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Drivetrain;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
@Disabled
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    private Vera m_vera = new Vera(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        m_vera.init(hardwareMap, true, false, CONSTANTS.FieldSide.LEFT,
                CONSTANTS.VeraPipelineType.SIGNAL, telemetry);
        Drivetrain drive = new Drivetrain(hardwareMap, m_vera);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}
