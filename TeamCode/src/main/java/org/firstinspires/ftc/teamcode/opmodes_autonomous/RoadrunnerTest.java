package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.middleware.VisionPipelineSignal.Signal;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskFindPole;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskReadSignal;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RoadRunner Test")
public class RoadrunnerTest extends LinOpAutonomousBase {

    private final TaskReadSignal m_taskReadSignal = new TaskReadSignal();
    private final TaskFindPole m_taskFindPole = new TaskFindPole(m_vera);

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

        waitForStart();

        m_vera.vision.startStreaming(VeraPipelineType.FIND_POLE);
        m_taskFindPole.setPoleType(PoleType.MID);


        m_taskFindPole.getOffsetToPole_deg();
        m_taskFindPole.getDistToScore_in();

        if (isStopRequested()) return;
        TrajectorySequence master = m_vera.drivetrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-24,2, Math.toRadians(-80)))
                .lineToSplineHeading(new Pose2d(-43.5,-4.5, Math.toRadians(-114)))
                .addTemporalMarker(2.2, () -> {m_vera.lift.moveLiftToMidPole();})
                .lineToSplineHeading(new Pose2d(-43.5,
                        m_taskFindPole.getDistToScore_in() - 4.5,
                        Math.toRadians(m_taskFindPole.getOffsetToPole_deg() - 114)))
                .waitSeconds(2)
                .addTemporalMarker(3, () -> m_vera.lift.dropCone())
                .build();

//        TrajectorySequence master = m_vera.drivetrain.trajectorySequenceBuilder(startPose)
//                .waitSeconds(1.0)
//                //.addTemporalMarker(1, () -> {m_vera.intake.setAutonomousInitDelayCount(12);})
//                //.addTemporalMarker(1, () -> {m_vera.lift.moveLiftToLowPole();})
//
//                // Preload cone
//                .lineToLinearHeading(new Pose2d(-24,2, Math.toRadians(-90)))
//                .lineToSplineHeading(new Pose2d(-37.5,-4, Math.toRadians(-90)))
//
//                .waitSeconds(1.5)
//                .addTemporalMarker(2.5, () -> {m_vera.lift.moveLiftToMidPole();})
//                .addTemporalMarker(3.5, () -> {m_vera.lift.dropCone();})
//                .addTemporalMarker(4.75, () -> {m_vera.lift.moveLiftToBottom();})
//                .addTemporalMarker(4.75, () -> {m_vera.intake.moveToIntakeConePos(5);})
//                .lineToLinearHeading(new Pose2d(-48,-4, Math.toRadians(-90)))
//                .lineToLinearHeading(new Pose2d(-48,-16, Math.toRadians(-90)))
//                .lineToSplineHeading(new Pose2d(-44,-2, Math.toRadians(-108)))
//                .addTemporalMarker(6.5, () -> {m_vera.lift.dropCone();})
//                .addTemporalMarker(7.75, () -> {m_vera.lift.moveLiftToBottom();})
//                .addTemporalMarker(7.75, () -> {m_vera.intake.moveToIntakeConePos(4);})
//
//
//
//
////                .lineToSplineHeading(new Pose2d(-47.75,18, Math.toRadians(-120)))
////                .addTemporalMarker(5, () -> {m_vera.lift.dropCone();})
////                .addTemporalMarker(7, () -> {m_vera.lift.moveLiftToBottom();})
//
//                // Stack cone 5
////                .addTemporalMarker(7.5, () -> {m_vera.intake.moveToIntakeConePos(5);})
////                .waitSeconds(2.75)
////                .lineToSplineHeading(new Pose2d(-48,0, Math.toRadians(-85)))
////                .lineToSplineHeading(new Pose2d(-48,-16, Math.toRadians(-90)))
////                .waitSeconds(2)
////                .lineToSplineHeading(new Pose2d(-47.75,18, Math.toRadians(-120)))
////                .addTemporalMarker(12, () -> {m_vera.lift.moveLiftToHighPole();})
//
//                // Stack cone 4
//
//                // Stack cone 3
//
//                // Stack cone 2
//
//                // Park in zone


        m_vera.drivetrain.followTrajectorySequenceAsync(master);

        while(!isStopRequested()) {
            getInputs();
            m_taskFindPole.update();
            commandVera();
            reportData();
        }
        stopVera();
    }
}

