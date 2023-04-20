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

@Autonomous(name = "Right Red 1+4 Mid")
public class RightRed1Plus4Mid extends LeftBlue1Plus4Mid {

    private final TaskReadSignal m_taskReadSignal = new TaskReadSignal();

    protected void initializeRoute() {
        setupAlliance(Alliance.RED, FieldSide.RIGHT);
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
        PreloadConeScorePos = new Pose2d(-49.2, 0.0, Math.toRadians(120));
        IntakePosCone5 = new Pose2d(-49.2, 18.7, Math.toRadians(94.5));
        ScoreCone5Pos = new Pose2d(-48.75, 2.0, Math.toRadians(124));
        IntakePosCone4 = new Pose2d(-48.5, 20.25, Math.toRadians(94.5));
        ScoreCone4Pos = new Pose2d(-48.75, 2.0, Math.toRadians(124));
        IntakePosCone3 = new Pose2d(-48.45, 22.25, Math.toRadians(94.5));
        ScoreCone3Pos = new Pose2d(-48.75, 2.0, Math.toRadians(124));
        IntakePosCone2 = new Pose2d(-47.6, 22.7, Math.toRadians(94.5));
        ScoreCone2Pos = new Pose2d(-48.75, 2.0, Math.toRadians(124));
        ParkZone1Pos = new Pose2d(-50.0, -21.0, Math.toRadians(90.0));
        ParkZone2Pos = new Pose2d(-49.0, 5.0, Math.toRadians(90.0));
        ParkZone3Pos = new Pose2d(-51.0, 33.5, Math.toRadians(90.0));

        runRoute();
    }
}