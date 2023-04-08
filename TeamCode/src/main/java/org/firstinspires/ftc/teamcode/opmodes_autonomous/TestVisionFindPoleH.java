package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.middleware.Vera;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskFindPole;

@TeleOp(name = "Test Vision Find Pole-H")
@Disabled
public class TestVisionFindPoleH extends LinearOpMode implements CONSTANTS {

    private Vera m_vera = new Vera();
    private final TaskFindPole m_taskFindPole = new TaskFindPole(m_vera);

    private boolean m_1DpadUp_AlreadyPressed = false;
    private boolean m_1DpadDown_AlreadyPressed = false;
    private boolean m_1DpadLeft_AlreadyPressed = false;
    private boolean m_1DpadRight_AlreadyPressed = false;

    private void initializeVera() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        m_vera.init(hardwareMap, false, true,
                VeraPipelineType.FIND_POLE, telemetry);

        if (Vera.isVisionTestMode) {
            telemetry.addData("WARNING:", "vision test mode!");
        } else {
            telemetry.addData("Status", "Initialized - " + m_vera.alliance);
        }
        telemetry.update();
    }

    private void getInputs() {
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
    }

    private void commandVera() {
        m_vera.commandVera();
    }

    private void reportData() {
        m_vera.reportData(telemetry);
        telemetry.update();
    }

    private void stopVera() {
        m_vera.stopVera();
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
            commandVera();
            reportData();
        } while ((status != TaskStatus.DONE) && !isStopRequested());

        stopVera();
    }
}
