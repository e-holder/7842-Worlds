package org.firstinspires.ftc.teamcode.opmodes_teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.middleware.Vera;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskFindPole;

@TeleOp(name = "Test Find Pole-Mid")
//@Disabled
public class TestFindPoleM extends LinearOpMode implements CONSTANTS {

    private Vera m_vera = new Vera(telemetry);
    private final TaskFindPole m_taskFindPole = new TaskFindPole(m_vera);

    private boolean m_1DpadUp_AlreadyPressed = false;
    private boolean m_1DpadDown_AlreadyPressed = false;
    private boolean m_1DpadLeft_AlreadyPressed = false;
    private boolean m_1DpadRight_AlreadyPressed = false;

    protected void initializeVera() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        m_vera.init(hardwareMap, false, true, FieldSide.LEFT,
                VeraPipelineType.FIND_POLE, telemetry);
        m_vera.setAllianceFromPriorAutonomousRun();

        telemetry.addData("WARNING:", "vision test mode!");
        telemetry.update();
    }

    protected void getInputs() {
        if (gamepad1.dpad_up && !m_1DpadUp_AlreadyPressed) {
            m_vera.vision.calBigStepUp();
        } else if (gamepad1.dpad_down && !m_1DpadDown_AlreadyPressed) {
            m_vera.vision.calBigStepDown();
        } else if (gamepad1.dpad_right && !m_1DpadRight_AlreadyPressed) {
            m_vera.vision.calSmallStepUp();
        } else if (gamepad1.dpad_left && !m_1DpadLeft_AlreadyPressed) {
            m_vera.vision.calSmallStepDown();

        }
        m_1DpadUp_AlreadyPressed = gamepad1.dpad_up;
        m_1DpadDown_AlreadyPressed = gamepad1.dpad_down;
        m_1DpadLeft_AlreadyPressed = gamepad1.dpad_left;
        m_1DpadRight_AlreadyPressed = gamepad1.dpad_right;

        m_vera.getInputs(false);
    }

    protected void startFindingPole(FindPoleMode findPoleMode) {
        m_taskFindPole.startFindingPole(findPoleMode, "");
    }

    protected void commandVera() {
        m_taskFindPole.update();  // Ignore return value
        m_vera.commandVera();
    }

    protected void reportData() {
        m_vera.reportData();
        telemetry.update();
    }

    protected void stopVera() {
        m_vera.stopVera();
    }

    protected void initializeFindPoleTask(FindPoleMode findPoleMode) {
        m_taskFindPole.setInitializationFindPoleMode(findPoleMode);
        TaskFindPole.TaskState taskState;
        do {
            getInputs();
            taskState = m_taskFindPole.update();
            commandVera();
            reportData();
        } while ((taskState != TaskFindPole.TaskState.IDLE) && !isStopRequested());
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initializeVera();
        initializeFindPoleTask(FindPoleMode.MID_POLE);

        waitForStart();

        startFindingPole(FindPoleMode.MID_POLE);

        do {
            getInputs();
            commandVera();
            reportData();
        } while (!isStopRequested());

        stopVera();
    }
}
