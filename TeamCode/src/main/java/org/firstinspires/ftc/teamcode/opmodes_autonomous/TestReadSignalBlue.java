package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskReadSignal;

@Autonomous(name = "Test Vision Read Signal Blue")
public class TestReadSignalBlue extends LinOpAutonomousBase implements CONSTANTS {

    private final TaskReadSignal m_taskReadSignal = new TaskReadSignal();

    @Override
    protected void preInitSetup() {
        m_isVisionTestMode = true;
        m_initialPipelineType = VeraPipelineType.SIGNAL;
    }

    @Override
    protected void initializeRoute() {
        setupAlliance(Alliance.BLUE, FieldSide.UNDEFINED);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initializeVera();

        TaskStatus status;
        do {
            getInputs();
            status = m_taskReadSignal.update();
            reportData();
        } while ((status != TaskStatus.DONE) && !isStopRequested());

        m_taskReadSignal.getParkingZone();  // Causes signal/parking data to be logged.

        waitForStart();

        while(!isStopRequested()) {
            getInputs();
            reportData();
        }
        stopVera();
    }
}

