package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.middleware.VisionPipelineSignal.Signal;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskFindPole;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskFindPole.TaskState;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskReadSignal;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

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

    private void startupFindPolePipeline(PoleType poleType) {
        m_vera.vision.startStreaming(VeraPipelineType.FIND_POLE);
        do {
            getInputs();
            m_taskFindPole.update();
            reportData();
        } while ((m_taskFindPole.getTaskState() != TaskState.FIND_POLE) && !isStopRequested());
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initializeVera();
        Signal parkingZone = readSignalCone();
        startupFindPolePipeline(PoleType.MID);

        Pose2d startPose = new Pose2d(0, 0, 0);
        m_vera.drivetrain.setPoseEstimate(startPose);

        waitForStart();

//        double x = m_vera.drivetrain.getPoseEstimate().getX();
//        double y = m_vera.drivetrain.getPoseEstimate().getY();
//        double heading = m_vera.drivetrain.getPoseEstimate().getHeading();
//        double distToPole = m_vera.vision.getDistToScore_in();
//        double headingToPole = m_vera.vision.getDeltaToPole_deg();

        if (isStopRequested()) return;
        TrajectorySequence master = m_vera.drivetrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-24.0,2.0, Math.toRadians(-80.0)))
                .lineToSplineHeading(new Pose2d(-43.5,-4.5, Math.toRadians(-114.0)))
                .addTemporalMarker(2.2, () -> {m_vera.lift.moveLiftToMidPole();})

//                .lineToSplineHeading(new Pose2d(-42.5, 0.5, Math.toRadians(-114.0)))

                .lineToSplineHeading(new Pose2d(
                        -42.5 + (m_vera.vision.getDistToScore_in() *
                                Math.sin(-114.0 - m_vera.vision.getDeltaToPole_deg())),
                        -4.5 + (m_vera.vision.getDistToScore_in() *
                                Math.cos(-114.0 - m_vera.vision.getDeltaToPole_deg())),
                        Math.toRadians(-114.0 - m_vera.vision.getDeltaToPole_deg())))

//                .lineToSplineHeading(new Pose2d(
//                        (m_vera.drivetrain.getPoseEstimate().getX() +
//                                (m_vera.vision.getDistToScore_in() *
//                                        Math.sin(m_vera.drivetrain.getPoseEstimate().getHeading() -
//                                                m_vera.vision.getDeltaToPole_deg()))),
//                        (m_vera.drivetrain.getPoseEstimate().getY() +
//                                (m_vera.vision.getDistToScore_in() *
//                                        Math.cos(m_vera.drivetrain.getPoseEstimate().getHeading() -
//                                                m_vera.vision.getDeltaToPole_deg()))),
//                        m_vera.drivetrain.getPoseEstimate().getHeading() -
//                                m_vera.vision.getDeltaToPole_deg()))

                .waitSeconds(0.5)
                .addTemporalMarker(3, () -> m_vera.lift.dropCone())
                .addTemporalMarker( 3.2,() -> m_vera.lift.moveLiftToBottom())
                .lineToSplineHeading(new Pose2d(-49, -12, Math.toRadians(-90)))
//                .waitSeconds(2)
//                .addTemporalMarker(6.5, () -> {m_vera.intake.moveToIntakeConePos(5);})


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


/*  Previous "master" branch when Davy was building a MID route
        TrajectorySequence master = m_vera.drivetrain.trajectorySequenceBuilder(startPose)
                .waitSeconds(1.0)
                //.addTemporalMarker(1, () -> {m_vera.intake.setAutonomousInitDelayCount(12);})
                //.addTemporalMarker(1, () -> {m_vera.lift.moveLiftToLowPole();})

                // Preload cone
                .lineToLinearHeading(new Pose2d(-24,2, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(-37.5,-4, Math.toRadians(-90)))

                .waitSeconds(1.5)
                .addTemporalMarker(2.5, () -> {m_vera.lift.moveLiftToMidPole();})
                .addTemporalMarker(3.5, () -> {m_vera.lift.dropCone();})
                .addTemporalMarker(4.75, () -> {m_vera.lift.moveLiftToBottom();})
                .addTemporalMarker(4.75, () -> {m_vera.intake.moveToIntakeConePos(5);})
                .lineToLinearHeading(new Pose2d(-48,-4, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-48,-16, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(-44,-2, Math.toRadians(-108)))
                .addTemporalMarker(6.5, () -> {m_vera.lift.dropCone();})
                .addTemporalMarker(7.75, () -> {m_vera.lift.moveLiftToBottom();})
                .addTemporalMarker(7.75, () -> {m_vera.intake.moveToIntakeConePos(4);})




//                .lineToSplineHeading(new Pose2d(-47.75,18, Math.toRadians(-120)))
//                .addTemporalMarker(5, () -> {m_vera.lift.dropCone();})
//                .addTemporalMarker(7, () -> {m_vera.lift.moveLiftToBottom();})

                // Stack cone 5
//                .addTemporalMarker(7.5, () -> {m_vera.intake.moveToIntakeConePos(5);})
//                .waitSeconds(2.75)
//                .lineToSplineHeading(new Pose2d(-48,0, Math.toRadians(-85)))
//                .lineToSplineHeading(new Pose2d(-48,-16, Math.toRadians(-90)))
//                .waitSeconds(2)
//                .lineToSplineHeading(new Pose2d(-47.75,18, Math.toRadians(-120)))
//                .addTemporalMarker(12, () -> {m_vera.lift.moveLiftToHighPole();})

                // Stack cone 4

                // Stack cone 3

                // Stack cone 2

                // Park in zone

                .build();

        m_vera.drivetrain.followTrajectorySequenceAsync(master);

*/
