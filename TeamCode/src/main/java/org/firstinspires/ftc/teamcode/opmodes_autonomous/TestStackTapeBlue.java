package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.AutonomousTask.TaskStatus;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskDetectStackTape;

@Autonomous(name = "Test Stack Tape Blue")
public class TestStackTapeBlue extends LinOpAutonomousBase implements CONSTANTS {

    private final TaskDetectStackTape m_taskDetectStackTape = new TaskDetectStackTape();

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
            getInputs();
            status = m_taskDetectStackTape.update();
            reportData();
        } while ((status != TaskStatus.DONE) && !isStopRequested());

//        m_taskReadSignal.getParkingZone();  // Causes signal/parking data to be logged.

        stopVera();
    }
}

