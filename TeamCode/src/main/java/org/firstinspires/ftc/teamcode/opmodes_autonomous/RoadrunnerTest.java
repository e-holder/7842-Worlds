package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.roadrunner.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.WaitSegment;

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

        //Positions for re-use
        //Pose2d IntakeLineupPos = new Pose2d(-49, -6, Math.toRadians(-90.0));
        Pose2d IntakePos = new Pose2d(-48, -18.5, Math.toRadians(-95.0));
        //Pose2d IntakePos = new Pose2d(-49, -20, Math.toRadians(-90.0));
        //Vector2d IntakeVec = new Vector2d(-49, -20);
        Pose2d ScorePos = new Pose2d(-48, -4, Math.toRadians(-122.5));
        double NudgeOffset = 23.0;

        //Velocities and Accelerations for re-use
        TrajectoryVelocityConstraint ScoringVelo = Drivetrain
                .getVelocityConstraint(32, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryVelocityConstraint IntakeVelo = Drivetrain
                .getVelocityConstraint(27, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint ConstAccel = Drivetrain
                .getAccelerationConstraint(DriveConstants.MAX_ACCEL);

        //Scoring Trajectories for re-use
        Trajectory PreloadTraj = m_vera.drivetrain.trajectoryBuilder(startPose)
                .lineToSplineHeading(ScorePos).build();
        Trajectory ScoreTraj = m_vera.drivetrain.trajectoryBuilder(IntakePos)
                .lineToSplineHeading(ScorePos, ScoringVelo, ConstAccel).build();

        //Wait Trajectories for re-use
        TrajectorySequence WaitForDrop = m_vera.drivetrain.trajectorySequenceBuilder(ScorePos).waitSeconds(1.5).build();
        TrajectorySequence WaitForDown = m_vera.drivetrain.trajectorySequenceBuilder(ScorePos).waitSeconds(0.5).build();
        TrajectorySequence WaitForIntake = m_vera.drivetrain.trajectorySequenceBuilder(IntakePos).waitSeconds(1.2).build();

        //Intake Trajectories for re-use
        Trajectory IntakeTraj = m_vera.drivetrain.trajectoryBuilder(ScorePos).lineToLinearHeading(IntakePos, IntakeVelo, ConstAccel).build();


        //Parking Trajectories

        waitForStart();

        if (isStopRequested()) return;

        //Scoring Preload
        m_vera.drivetrain.followTrajectory(PreloadTraj);
        m_vera.drivetrain.findPole(FindPoleMode.MID_POLE, "preload");
        m_vera.lift.moveLiftToMidPole();
        m_vera.intake.moveToIdlePos();
        m_vera.drivetrain.followTrajectorySequence(WaitForDrop);
        m_vera.lift.dropCone();
        m_vera.drivetrain.stopFindingPole();
        m_vera.drivetrain.followTrajectorySequence(WaitForDown);
        m_vera.lift.moveLiftToBottom();

        //Stack Cone 5
        m_vera.intake.turnOnStackTapeSensing();
        m_vera.drivetrain.followTrajectory(IntakeTraj);
        m_vera.intake.moveToIntakeConePos(5);
        m_vera.drivetrain.followTrajectorySequence(WaitForIntake);
        m_vera.drivetrain.followTrajectory(ScoreTraj);
        m_vera.drivetrain.findPole(FindPoleMode.MID_POLE, "cone5");
        m_vera.lift.moveLiftToMidPole();
        m_vera.intake.moveToIdlePos();
        m_vera.drivetrain.followTrajectorySequence(WaitForDrop);
        m_vera.lift.dropCone();
        m_vera.drivetrain.stopFindingPole();
        m_vera.drivetrain.followTrajectorySequence(WaitForDown);
        m_vera.lift.moveLiftToBottom();

        //Stack Cone 4
        m_vera.intake.turnOnStackTapeSensing();
        m_vera.drivetrain.followTrajectory(IntakeTraj);
        m_vera.intake.moveToIntakeConePos(4);
        m_vera.drivetrain.followTrajectorySequence(WaitForIntake);
        m_vera.drivetrain.followTrajectory(ScoreTraj);
        m_vera.drivetrain.findPole(FindPoleMode.MID_SCORED_CONES, "cone4");
        m_vera.lift.moveLiftToMidPole();
        m_vera.intake.moveToIdlePos();
        m_vera.drivetrain.followTrajectorySequence(WaitForDrop);
        m_vera.lift.dropCone();
        m_vera.drivetrain.stopFindingPole();
        m_vera.drivetrain.followTrajectorySequence(WaitForDown);
        m_vera.lift.moveLiftToBottom();

        //Stack Cone 3
        m_vera.intake.turnOnStackTapeSensing();
        m_vera.drivetrain.followTrajectory(IntakeTraj);
        m_vera.intake.moveToIntakeConePos(3);
        m_vera.drivetrain.followTrajectorySequence(WaitForIntake);
        m_vera.drivetrain.followTrajectory(ScoreTraj);
        m_vera.drivetrain.findPole(FindPoleMode.MID_SCORED_CONES, "cone3");
        m_vera.lift.moveLiftToMidPole();
        m_vera.intake.moveToIdlePos();
        m_vera.drivetrain.followTrajectorySequence(WaitForDrop);
        m_vera.lift.dropCone();
        m_vera.drivetrain.stopFindingPole();
        m_vera.drivetrain.followTrajectorySequence(WaitForDown);
        m_vera.lift.moveLiftToBottom();

        //Stack Cone 2
        m_vera.intake.turnOnStackTapeSensing();
        m_vera.drivetrain.followTrajectory(IntakeTraj);
        m_vera.intake.moveToIntakeConePos(2);
        m_vera.drivetrain.followTrajectorySequence(WaitForIntake);
        m_vera.drivetrain.followTrajectory(ScoreTraj);
        m_vera.drivetrain.findPole(FindPoleMode.MID_SCORED_CONES, "cone2");
        m_vera.lift.moveLiftToMidPole();
        m_vera.intake.moveToIdlePos();
        m_vera.drivetrain.followTrajectorySequence(WaitForDrop);
        m_vera.lift.dropCone();
        m_vera.drivetrain.stopFindingPole();
        m_vera.drivetrain.followTrajectorySequence(WaitForDown);
        m_vera.lift.moveLiftToBottom();

        //Stack Cone 1
        m_vera.intake.turnOnStackTapeSensing();
        m_vera.drivetrain.followTrajectory(IntakeTraj);
        m_vera.intake.moveToIntakeConePos(1);
        m_vera.drivetrain.followTrajectorySequence(WaitForIntake);
        m_vera.drivetrain.followTrajectory(ScoreTraj);
        m_vera.drivetrain.findPole(FindPoleMode.MID_SCORED_CONES, "cone1");
        m_vera.lift.moveLiftToMidPole();
        m_vera.intake.moveToIdlePos();
        m_vera.drivetrain.followTrajectorySequence(WaitForDrop);
        m_vera.lift.dropCone();
        m_vera.drivetrain.stopFindingPole();
        m_vera.drivetrain.followTrajectorySequence(WaitForDown);
        m_vera.lift.moveLiftToBottom();

        //Park in zone

        stopVera();
    }
}