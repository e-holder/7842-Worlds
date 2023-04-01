package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.AutonomousTask;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskDelay;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskReadSignal;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;



@Autonomous(name = "Test Vision Read Signal Blue")
public class TestReadSignalBlue extends LinOpAutonomousBase implements CONSTANTS {

    // TODO: How to get these tasks sequenced in new architecture?
    private TaskReadSignal m_taskReadSignal = new TaskReadSignal(true);
    private TaskDelay m_taskDelay = new TaskDelay(8000);

    @Override
    protected void preInitSetup() {
        m_isVisionTestMode = true;
        m_initialPipelineType = VeraPipelineType.SIGNAL;
    }

    @Override
    protected void initializeRoute() {
        setupAlliance(Alliance.BLUE, FieldSide.SOUTH);

        // TODO: How to get tasks (see lines 14-15) sequenced in new architecture?
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initializeVera();

        waitForStart();

        if (isStopRequested()) return;

        boolean running = true;
        int task = 0;

        while(running && !isStopRequested()) {
            m_vera.getInputs(true);
            switch (task) {
                case 0:
                    if (m_taskReadSignal.update(0) == TaskStatus.DONE) {
                        task = 1;
                    }
                    break;
                case 1:
                    if (m_taskDelay.update(0) == TaskStatus.DONE) {
                        running = false;
                    }
                    break;
            }
            m_vera.commandVera();
        }
        m_vera.stopVera();
    }

}

