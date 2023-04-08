package org.firstinspires.ftc.teamcode.opmodes_teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.middleware.Intake;
import org.firstinspires.ftc.teamcode.middleware.Vera;

@TeleOp(name="Test Calibrate Stack Tape")
//@Disabled
public class LinOpTestStackTape extends LinearOpMode implements CONSTANTS {

    private Vera m_vera = new Vera();

    private void initializeVera() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        m_vera.init(hardwareMap, false, false,
                VeraPipelineType.SIGNAL, telemetry);
        m_vera.setAllianceFromPriorAutonomousRun();
        m_vera.intake.enableStackTapeCalibrationMode();

        m_vera.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if (Vera.isVisionTestMode) {
            telemetry.addData("WARNING:", "vision test mode!");
        } else {
            telemetry.addData("Status", "Initialized - " + m_vera.alliance);
        }
        telemetry.update();
    }

    private void getInputs() {
        // getInputs MUST be the first thing called in this function.
        m_vera.getInputs(false);
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
    public void runOpMode() {
        initializeVera();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        m_vera.intake.turnOnStackTapeSensing();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            getInputs();
            commandVera();
            reportData();
        }

        m_vera.intake.turnOffStackTapeSensing();

        stopVera();
    }
}

