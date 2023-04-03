package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.AutonomousTask.TaskStatus;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskDelay;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskReadSignal;

@Autonomous(name = "Test Vision Read Signal Blue")
public class TestReadSignalBlue extends LinOpAutonomousBase implements CONSTANTS {

    private TaskReadSignal m_taskReadSignal = new TaskReadSignal();
    private TaskDelay m_taskDelay = new TaskDelay(30000);

    @Override
    protected void preInitSetup() {
        m_isVisionTestMode = true;
        m_initialPipelineType = VeraPipelineType.SIGNAL;
    }

    @Override
    protected void initializeRoute() {
        setupAlliance(Alliance.BLUE, FieldSide.SOUTH);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initializeVera();

        waitForStart();

        if (isStopRequested()) return;

        boolean running = true;
        int task = 0;

        while(running && !isStopRequested()) {
            getInputs();

            switch (task) {
                case 0:
                    if (m_taskReadSignal.update() == TaskStatus.DONE) {
                        task = 1;
                    }
                    break;
                case 1:
                    if (m_taskDelay.update() == TaskStatus.DONE) {
                        running = false;
                    }
                    break;
            }
            m_taskReadSignal.displaySignalTelemetry();

            commandVera();
            reportData();
        }
        stopVera();
    }

}

