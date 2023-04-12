package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.middleware.Intake;
import org.firstinspires.ftc.teamcode.middleware.VisionPipelineSignal.Signal;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskFindPole;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskReadSignal;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RoadRunner Test")
public class RoadrunnerTest extends LinOpAutonomousBase {

    private final TaskReadSignal m_taskReadSignal = new TaskReadSignal();

    protected void initializeRoute() {
        setupAlliance(Alliance.BLUE, FieldSide.LEFT);
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

        initializeVera();
        Signal parkingZone = readSignalCone();

        Pose2d startPose = new Pose2d(0, 0, 0);
        m_vera.drivetrain.setPoseEstimate(startPose);

        //Positions
        Pose2d IntakePos = new Pose2d(-50.5, -18.5, Math.toRadians(-90.0));
        Pose2d ScorePos = new Pose2d(-50, -4, Math.toRadians(-135.0));
        TrajectoryVelocityConstraint ScoringVelo = m_vera.drivetrain.getVelocityConstraint(32, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryVelocityConstraint IntakeVelo = m_vera.drivetrain.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint ConstAccel = m_vera.drivetrain.getAccelerationConstraint(DriveConstants.MAX_ACCEL);

        //Scoring Trajectory
        Trajectory PreloadTraj = m_vera.drivetrain.trajectoryBuilder(startPose).lineToSplineHeading(ScorePos).build();
        Trajectory ScoreTraj = m_vera.drivetrain.trajectoryBuilder(IntakePos).lineToSplineHeading(ScorePos, ScoringVelo, ConstAccel).build();

        //Wait Trajectory
        TrajectorySequence WaitForDrop = m_vera.drivetrain.trajectorySequenceBuilder(ScorePos).waitSeconds(1.5).build();
        TrajectorySequence WaitForDown = m_vera.drivetrain.trajectorySequenceBuilder(ScorePos).waitSeconds(0.5).build();
        TrajectorySequence WaitForIntake = m_vera.drivetrain.trajectorySequenceBuilder(IntakePos).waitSeconds(0.33).build();

        //Intake Trajectories
        Trajectory IntakeTraj = m_vera.drivetrain.trajectoryBuilder(ScorePos).lineToSplineHeading(IntakePos, IntakeVelo, ConstAccel).build();

        //Parking Trajectories

        waitForStart();

        if (isStopRequested()) return;

        //Scoring Preload
        m_vera.drivetrain.followTrajectory(PreloadTraj);
        m_vera.drivetrain.findMidPole();
        m_vera.lift.moveLiftToMidPole();
        m_vera.drivetrain.followTrajectorySequence(WaitForDrop);
        m_vera.lift.dropCone();
        m_vera.drivetrain.stopFindingPole();
        m_vera.drivetrain.followTrajectorySequence(WaitForDown);
        m_vera.lift.moveLiftToBottom();

        //Stack Cone 5
        m_vera.drivetrain.followTrajectory(IntakeTraj);
        m_vera.intake.moveToIntakeConePos(5);
        m_vera.drivetrain.followTrajectorySequence(WaitForIntake);
        m_vera.drivetrain.followTrajectory(ScoreTraj);
        m_vera.drivetrain.findMidPole();
        m_vera.lift.moveLiftToMidPole();
        m_vera.drivetrain.followTrajectorySequence(WaitForDrop);
        m_vera.lift.dropCone();
        m_vera.drivetrain.stopFindingPole();
        m_vera.drivetrain.followTrajectorySequence(WaitForDown);
        m_vera.lift.moveLiftToBottom();



        stopVera();
    }
}