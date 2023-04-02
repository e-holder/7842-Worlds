package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.middleware.Vera;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;


@Autonomous
public class RoadrunnerTest extends LinOpAutonomousBase {
    @Override
    public void runOpMode() throws InterruptedException {

        initializeVera();

        Pose2d startPose = new Pose2d(0, 0, 0);

        m_vera.drivetrain.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence master = m_vera.drivetrain.trajectorySequenceBuilder(startPose)
                .waitSeconds(1.0)
                .addTemporalMarker(1, () -> {m_vera.intake.setAutonomousInitDelayCount(12);})
                //.addTemporalMarker(1, () -> {m_vera.lift.moveLiftToLowPole();})

                //Preload
                .lineToSplineHeading(new Pose2d(-56,-1, Math.toRadians(-120)))
                .waitSeconds(0.33)
                .addTemporalMarker(3, () -> {m_vera.lift.moveLiftToHighPole();})
                .lineToSplineHeading(new Pose2d(-47.75,18, Math.toRadians(-120)))
                .addTemporalMarker(5, () -> {m_vera.lift.dropCone();})
                .addTemporalMarker(7, () -> {m_vera.lift.moveLiftToBottom();})
                .addTemporalMarker(7.5, () -> {m_vera.intake.moveToIntakeConePos(5);})
                .waitSeconds(2.75)
                .lineToSplineHeading(new Pose2d(-48,0, Math.toRadians(-85)))
                .lineToSplineHeading(new Pose2d(-48,-16, Math.toRadians(-90)))
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(-47.75,18, Math.toRadians(-120)))
                .addTemporalMarker(12, () -> {m_vera.lift.moveLiftToHighPole();})



                .build();

        m_vera.drivetrain.followTrajectorySequenceAsync(master);
        while(!isStopRequested()) {
            m_vera.drivetrain.update();
            m_vera.getInputs(true);
            m_vera.commandVera();
            m_vera.reportData(telemetry);
        }

        stopVera();
    }

    protected void initializeRoute() {
        setupAlliance(Alliance.BLUE, FieldSide.SOUTH);

        // TODO: How to get tasks (see lines 14-15) sequenced in new architecture?
    }
}

