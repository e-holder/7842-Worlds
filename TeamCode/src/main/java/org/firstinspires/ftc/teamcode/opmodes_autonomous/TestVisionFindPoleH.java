package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.middleware.Vision;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.AutonomousTask.TaskStatus;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskFindPole;

@Autonomous(name = "Test Vision Find Pole-H")
//@Disabled
public class TestVisionFindPoleH extends LinOpAutonomousBase implements CONSTANTS {

    private TaskFindPole m_taskFindPole = new TaskFindPole();

    @Override
    protected void preInitSetup() {
        m_isVisionTestMode = true;
        m_initialPipelineType = VeraPipelineType.FIND_POLE;
    }

    @Override
    protected void initializeRoute() {
        setupAlliance(Alliance.BLUE, FieldSide.UNDEFINED);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initializeVera();
        m_taskFindPole.setPoleType(PoleType.HIGH);

        waitForStart();

        TaskStatus status;
        do {
            getInputs();
            status = m_taskFindPole.update();
            reportData();
        } while ((status != TaskStatus.DONE) && !isStopRequested());

        stopVera();
    }
}
