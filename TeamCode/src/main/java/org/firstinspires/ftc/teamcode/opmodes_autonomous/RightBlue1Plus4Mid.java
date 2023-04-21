package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.middleware.VisionPipelineSignal.Signal;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskReadSignal;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Right Blue 1+4 Mid")
public class RightBlue1Plus4Mid extends LeftBlue1Plus4Mid {

    protected void initializeRoute() {
        setupAlliance(Alliance.BLUE, FieldSide.RIGHT);
    }

    protected void setPositions() {
        //Positions in order of route
        startPose = new Pose2d(0, 0, 0);
        PreloadConeScorePos = new Pose2d(-49.3, 0.0, Math.toRadians(120));
        IntakePosCone5 = new Pose2d(-49.2, 17.8, Math.toRadians(94.5));
        ScoreCone5Pos = new Pose2d(-48.35, 1.0, Math.toRadians(124));
        IntakePosCone4 = new Pose2d(-48.5, 18.95, Math.toRadians(94.5));
        ScoreCone4Pos = new Pose2d(-47.15, 0.5, Math.toRadians(124));
        IntakePosCone3 = new Pose2d(-48.45, 21.15, Math.toRadians(94.5));
        ScoreCone3Pos = new Pose2d(-46.25, 2.0, Math.toRadians(124));
        IntakePosCone2 = new Pose2d(-47.6, 21.6, Math.toRadians(94.5));
        ScoreCone2Pos = new Pose2d(-45.75, 2.5, Math.toRadians(124));
        ParkZone1Pos = new Pose2d(-51.25, -23.0, Math.toRadians(90.0));
        ParkZone2Pos = new Pose2d(-49.0, 2.0, Math.toRadians(90.0));
        ParkZone3Pos = new Pose2d(-51.0, 33.5, Math.toRadians(90.0));
        m_vera.drivetrain.setPoseEstimate(startPose);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        boolean m_moveIntakeIfNoCone = true;

        initializeVera();
        readSignalCone();  // Ignore return value. This just initializes the reading process.
        setPositions();
        runRoute();
    }
}