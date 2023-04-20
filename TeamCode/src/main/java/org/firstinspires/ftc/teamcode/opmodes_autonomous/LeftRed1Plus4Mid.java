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

@Autonomous(name = "Left Red 1+4 Mid")
public class LeftRed1Plus4Mid extends LeftBlue1Plus4Mid {

    private final TaskReadSignal m_taskReadSignal = new TaskReadSignal();

    protected void initializeRoute() {
        setupAlliance(Alliance.RED, FieldSide.LEFT);
    }

    // Use vision (webcam) to read the signal cone.
    private Signal readSignalCone() {
        TaskStatus status;
        do {
            getInputs();
            status = m_taskReadSignal.update();
            reportData();
        } while ((status != TaskStatus.DONE) && !isStopRequested());
        return m_taskReadSignal.getParkingZone();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        boolean m_moveIntakeIfNoCone = true;

        initializeVera();
        Signal parkingZone = readSignalCone();

        Pose2d startPose = new Pose2d(0, 0, 0);
        m_vera.drivetrain.setPoseEstimate(startPose);

        //Positions in order of route
        PreloadConeScorePos = new Pose2d(-48.5, -2.0, Math.toRadians(-120));
        IntakePosCone5 = new Pose2d(-49.2, -19.3, Math.toRadians(-94.5));
        ScoreCone5Pos = new Pose2d(-48.75, -2.0, Math.toRadians(-124));
        IntakePosCone4 = new Pose2d(-48.5, -19.35, Math.toRadians(-94.5));
        ScoreCone4Pos = new Pose2d(-48.75, -2.0, Math.toRadians(-124));
        IntakePosCone3 = new Pose2d(-48.45, -19.45, Math.toRadians(-94.5));
        ScoreCone3Pos = new Pose2d(-48.75, -2.0, Math.toRadians(-124));
        IntakePosCone2 = new Pose2d(-47.6, -18.7, Math.toRadians(-94.5));
        ScoreCone2Pos = new Pose2d(-48.75, -2.0, Math.toRadians(-124));
        ParkZone1Pos = new Pose2d(-47.5, -27.0, Math.toRadians(-90.0));
        ParkZone2Pos = new Pose2d(-49.0, -3.0, Math.toRadians(-90.0));
        ParkZone3Pos = new Pose2d(-48.0, 22.5, Math.toRadians(-90.0));

        runRoute();
    }
}