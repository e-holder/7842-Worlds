package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskDelay;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskReadSignal;

@Autonomous(name = "Test Vision Read Signal Red")
//@Disabled
public class TestReadSignalRed extends LinOpAutonomousBase implements CONSTANTS {

    private TaskReadSignal m_taskReadSignal = new TaskReadSignal();
    private TaskDelay m_taskDelay = new TaskDelay(8000);

    @Override
    protected void preInitSetup() {
        m_isVisionTestMode = true;
        m_initialPipelineType = VeraPipelineType.SIGNAL;
    }

    @Override
    protected void initializeRoute() {
        setupAlliance(Alliance.RED, FieldSide.SOUTH);

        // TODO: How to get tasks (see lines 14-15) sequenced in new architecture?
    }
}
