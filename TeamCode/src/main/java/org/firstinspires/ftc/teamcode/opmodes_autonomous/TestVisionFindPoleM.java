package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.middleware.Vision;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.AutonomousTask.TaskStatus;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskFindPole;

@Autonomous(name = "Test Vision Find Pole-M")
//@Disabled
public class TestVisionFindPoleM extends LinOpAutonomousBase implements CONSTANTS {

    private TaskFindPole m_taskFindPole = new TaskFindPole(PoleType.MID);

    @Override
    protected void preInitSetup() {
        m_isVisionTestMode = true;
        m_initialPipelineType = VeraPipelineType.FIND_POLE;
        Vision.poleType = PoleType.MID;
    }

    @Override
    protected void initializeRoute() {
        setupAlliance(Alliance.BLUE, FieldSide.UNDEFINED);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initializeVera();

        waitForStart();

        TaskStatus status;
        do {
            status = m_taskFindPole.update();
            m_taskFindPole.addFindPoleTelemetry();
            reportData();
        } while ((status != TaskStatus.DONE) && !isStopRequested());

        stopVera();
    }
}
