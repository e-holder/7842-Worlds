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

@Autonomous(name = "Right Blue 1+3 Mid")
public class RightBlue1Plus3Mid extends LinOpAutonomousBase {

    private final TaskReadSignal m_taskReadSignal = new TaskReadSignal();

    protected void initializeRoute() {
        setupAlliance(Alliance.BLUE, FieldSide.RIGHT);
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

        //Positions in order of route
        Pose2d PreloadConeScorePos = new Pose2d(-48.2, 0.0, Math.toRadians(120));
        Pose2d IntakePosCone5 = new Pose2d(-49.2, 19.1, Math.toRadians(94.5));
        Pose2d ScoreConePos = new Pose2d(-48.75, 2.0, Math.toRadians(124));
        Pose2d IntakePosCone4 = new Pose2d(-48.5, 20.65, Math.toRadians(94.5));
        //ScoreConePos TODO: make different traj for all cone scoring
        Pose2d IntakePosCone3 = new Pose2d(-48.45, 22.25, Math.toRadians(94.5));
        //ScoreConePos TODO: make different traj for all cone scoring
        Pose2d IntakePosCone2 = new Pose2d(-47.6, 22.7, Math.toRadians(94.5));
        Pose2d ScoreCone2Pos = new Pose2d(-48.75, 3.25, Math.toRadians(124.0));
        //ScoreConePos TODO: make different traj for all cone scoring
        Pose2d ParkZone1Pos = new Pose2d(-53.0, -23.0, Math.toRadians(90.0));
        Pose2d ParkZone2Pos = new Pose2d(-49.0, 5.0, Math.toRadians(90.0));
        Pose2d ParkZone3Pos = new Pose2d(-51.0, 33.5, Math.toRadians(90.0));

        //Velocities and Accelerations for re-use
        TrajectoryVelocityConstraint ScoringVelo = Drivetrain
                .getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryVelocityConstraint IntakeVelo = Drivetrain
                .getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryVelocityConstraint ParkVelo = Drivetrain
                .getVelocityConstraint(90, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint ConstAccel = Drivetrain
                .getAccelerationConstraint(DriveConstants.MAX_ACCEL);

        //Scoring Trajectories for re-use
        Trajectory PreloadTraj = m_vera.drivetrain.trajectoryBuilder(startPose)
                .lineToSplineHeading(PreloadConeScorePos).build();
        Trajectory ScoreConeTraj = m_vera.drivetrain.trajectoryBuilder(IntakePosCone5)
                .lineToSplineHeading(ScoreConePos, ScoringVelo, ConstAccel).build();
        Trajectory ScoreCone2Traj = m_vera.drivetrain.trajectoryBuilder(IntakePosCone5)
                .lineToSplineHeading(ScoreCone2Pos, ScoringVelo, ConstAccel).build();

        //Wait Trajectories for re-use
        TrajectorySequence WaitForDrop = m_vera.drivetrain
                .trajectorySequenceBuilder(PreloadConeScorePos).waitSeconds(0.95).build();
        TrajectorySequence WaitForDown = m_vera.drivetrain
                .trajectorySequenceBuilder(PreloadConeScorePos).waitSeconds(0.2).build();
        TrajectorySequence WaitForIntake = m_vera.drivetrain
                .trajectorySequenceBuilder(IntakePosCone5).waitSeconds(0.65).build();
        TrajectorySequence WaitToPark = m_vera.drivetrain
                .trajectorySequenceBuilder(ScoreConePos).waitSeconds(2.0).build();

        //Intake Trajectories for re-use
        Trajectory IntakeCone5Traj = m_vera.drivetrain
                .trajectoryBuilder(PreloadConeScorePos)
                .lineToLinearHeading(IntakePosCone5, IntakeVelo, ConstAccel).build();
        Trajectory IntakeCone4Traj = m_vera.drivetrain
                .trajectoryBuilder(ScoreConePos)
                .lineToLinearHeading(IntakePosCone4, IntakeVelo, ConstAccel).build();
        Trajectory IntakeCone3Traj = m_vera.drivetrain
                .trajectoryBuilder(ScoreConePos)
                .lineToLinearHeading(IntakePosCone3, IntakeVelo, ConstAccel).build();
        Trajectory IntakeCone2Traj = m_vera.drivetrain
                .trajectoryBuilder(ScoreConePos)
                .lineToLinearHeading(IntakePosCone2, IntakeVelo, ConstAccel).build();

        //Parking Trajectories
        Trajectory ParkZone1Traj = m_vera.drivetrain.trajectoryBuilder(ScoreConePos)
                .lineToLinearHeading(ParkZone1Pos, ParkVelo, ConstAccel).build();
        Trajectory ParkZone2Traj = m_vera.drivetrain.trajectoryBuilder(ScoreConePos)
                .lineToLinearHeading(ParkZone2Pos, ParkVelo, ConstAccel).build();
        Trajectory ParkZone3Traj = m_vera.drivetrain.trajectoryBuilder(ScoreConePos)
                .lineToLinearHeading(ParkZone3Pos, ParkVelo, ConstAccel).build();
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
        m_vera.lift.moveLiftToBottom();

        //Stack Cone 5
        m_vera.intake.turnOnStackTapeSensing();
        m_vera.intake.moveToIntakeConePos(5);
        m_vera.intake.turnOffStackTapeSensing();
        m_vera.drivetrain.followTrajectory(IntakeCone5Traj);
        m_vera.drivetrain.followTrajectorySequence(WaitForIntake);
        m_vera.drivetrain.followTrajectory(ScoreConeTraj);
        m_vera.drivetrain.findPole(FindPoleMode.MID_POLE, "cone5");
        m_vera.lift.moveLiftToMidPole();
        m_vera.intake.moveToIdlePos();
        m_vera.drivetrain.followTrajectorySequence(WaitForDrop);
        m_vera.lift.dropCone();
        m_vera.drivetrain.stopFindingPole();
        m_vera.lift.moveLiftToBottom();

        //Stack Cone 4
        m_vera.intake.turnOnStackTapeSensing();
        m_vera.intake.moveToIntakeConePos(4);
        m_vera.intake.turnOffStackTapeSensing();
        m_vera.drivetrain.followTrajectory(IntakeCone4Traj);
        m_vera.drivetrain.followTrajectorySequence(WaitForIntake);
        m_vera.drivetrain.followTrajectory(ScoreConeTraj);
        m_vera.drivetrain.findPole(FindPoleMode.MID_POLE, "cone4");
        m_vera.lift.moveLiftToMidPole();
        m_vera.intake.moveToIdlePos();
        m_vera.drivetrain.followTrajectorySequence(WaitForDrop);
        m_vera.lift.dropCone();
        m_vera.drivetrain.stopFindingPole();
        m_vera.lift.moveLiftToBottom();
        m_vera.vision.setFindPoleMode(FindPoleMode.MID_SCORED_CONES, "");

        //Stack Cone 3
        m_vera.intake.turnOnStackTapeSensing();
        m_vera.intake.moveToIntakeConePos(3);
        m_vera.intake.turnOffStackTapeSensing();
        m_vera.drivetrain.followTrajectory(IntakeCone3Traj);
        m_vera.drivetrain.followTrajectorySequence(WaitForIntake);
        m_vera.drivetrain.followTrajectory(ScoreCone2Traj);
        m_vera.drivetrain.findPole(FindPoleMode.MID_SCORED_CONES, "cone3");
        m_vera.lift.moveLiftToMidPole();
        m_vera.intake.moveToIdlePos();
        m_vera.drivetrain.followTrajectorySequence(WaitForDrop);
        m_vera.lift.dropCone();
        m_vera.drivetrain.stopFindingPole();
        m_vera.lift.moveLiftToBottom();

        //Park in zone
        switch (parkingZone) {
            case ZONE1:
                m_vera.drivetrain.followTrajectorySequence(WaitToPark);
                m_vera.drivetrain.followTrajectory(ParkZone1Traj);
                break;
            case ZONE2:
                m_vera.drivetrain.followTrajectory(ParkZone2Traj);
                break;
            case ZONE3:
                m_vera.drivetrain.followTrajectory(ParkZone3Traj);
                break;
        }

        stopVera();
    }
}