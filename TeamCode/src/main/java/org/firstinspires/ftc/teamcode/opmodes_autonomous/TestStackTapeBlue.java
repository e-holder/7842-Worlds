package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
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

        m_taskDetectStackTape.turnOffStackTapeSensing();

        // These calls cause this data to be logged by the intake.
        double deltaX_in = m_taskDetectStackTape.getStackDeltaX_in();
        double deltaHeading_deg = m_taskDetectStackTape.getStackDeltaHeading_deg();

        stopVera();
    }
}

