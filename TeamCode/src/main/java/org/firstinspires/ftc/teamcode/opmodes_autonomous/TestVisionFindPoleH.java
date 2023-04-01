package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.middleware.Vision;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskTestVisionFindPole;

@Autonomous(name = "Test Vision Find Pole-H")
//@Disabled
public class TestVisionFindPoleH extends LinOpAutonomousBase implements CONSTANTS {

    // TODO: How to get tasks sequenced in new architecture?
    private TaskTestVisionFindPole m_taskTestVisionFindPole =
            new TaskTestVisionFindPole(CornerType.H, true);

    @Override
    protected void preInitSetup() {
        m_isVisionTestMode = true;
        m_initialPipelineType = VeraPipelineType.FIND_POLE;
        Vision.poleType = CornerType.H;
    }

    @Override
    protected void initializeRoute() {
        setupAlliance(Alliance.BLUE, FieldSide.NORTH);

        // TODO: How to get tasks (line 14) sequenced in new architecture?
    }
}
