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
                .addTemporalMarker(1, () -> {m_vera.lift.moveLiftToLowPole();})

                //Preload
                .back(42.0)
                .addTemporalMarker(3, () -> {m_vera.lift.moveLiftToHighPole();})
                .lineToLinearHeading(new Pose2d(-50.5,-11.5, Math.toRadians(-61.5)))
                .waitSeconds(1)
                .back(3.0)
                .addTemporalMarker(5.25, () -> {m_vera.lift.dropCone();})
                .addTemporalMarker(6.25, () -> {m_vera.lift.moveLiftToBottom();})
                .addTemporalMarker(6.25, () -> {m_vera.intake.moveToIntakeConePos(4);})
                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(-48.5,-11.5, Math.toRadians(-90)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-48.5,-15.5, Math.toRadians(-90)))

                //1st stack cone
                .addTemporalMarker(8.5, () -> {m_vera.lift.moveLiftToLowPole();})
                .addTemporalMarker(9.5, () -> {m_vera.lift.moveLiftToHighPole();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-50.5,-11.5, Math.toRadians(-61.5)))
                .waitSeconds(2)
                .back(3.0)
                .addTemporalMarker(11.75, () -> {m_vera.lift.dropCone();})
                .addTemporalMarker(13.25, () -> {m_vera.lift.moveLiftToBottom();})
                .addTemporalMarker(13.25, () -> {m_vera.intake.moveToIntakeConePos(4);})
                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(-50.5,-11.5, Math.toRadians(-90)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-48.5,-15.5, Math.toRadians(-90)))

                //2nd stack cone
                .addTemporalMarker(15.5, () -> {m_vera.lift.moveLiftToLowPole();})
                .addTemporalMarker(16.5, () -> {m_vera.lift.moveLiftToHighPole();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-50.5,-11.5, Math.toRadians(-61.5)))
                .waitSeconds(2)
                .back(3.0)
                .addTemporalMarker(18.75, () -> {m_vera.lift.dropCone();})
                .addTemporalMarker(20.25, () -> {m_vera.lift.moveLiftToBottom();})
                .addTemporalMarker(20.25, () -> {m_vera.intake.moveToIntakeConePos(4);})
                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(-50.5,-11.5, Math.toRadians(-90)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-48.5,-15.5, Math.toRadians(-90)))

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

