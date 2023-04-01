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
                .addTemporalMarker(1, () -> {
                    m_vera.intake.setAutonomousInitDelayCount(12);
                    m_vera.lift.moveLiftToLowPole();
                })
                .back(42.0)
                .addTemporalMarker(3, () -> {
                    m_vera.lift.moveLiftToHighPole();
                })
                .lineToLinearHeading(new Pose2d(-50.5,-11.5, Math.toRadians(-62)))
                .addTemporalMarker(5.25, () -> {
                    m_vera.lift.dropCone();
                    m_vera.lift.moveLiftToBottom();
//                    m_vera.intake.
                })
                .waitSeconds(3.0)
                .turn(Math.toRadians(-28))
                .forward(11.0)
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

