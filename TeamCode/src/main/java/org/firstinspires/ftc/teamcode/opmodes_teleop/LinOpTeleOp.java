package org.firstinspires.ftc.teamcode.opmodes_teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.middleware.Vera;

@TeleOp(name = "TeleOp")
public class LinOpTeleOp extends LinearOpMode implements CONSTANTS {

    private Vera m_vera = new Vera();

    private boolean m_1DpadUp_AlreadyPressed = false;
    private boolean m_1DpadRight_AlreadyPressed = false;
    private boolean m_1LeftBumper_AlreadyPressed = false;
    private boolean m_1RightBumper_AlreadyPressed = false;
    private boolean m_1A_AlreadyPressed = false;
    private boolean m_1B_AlreadyPressed = false;
    private boolean m_1X_AlreadyPressed = false;
    private boolean m_1Y_AlreadyPressed = false;

    private boolean m_2DpadUp_AlreadyPressed = false;
    private boolean m_2DpadRight_AlreadyPressed = false;
    private boolean m_2LeftBumper_AlreadyPressed = false;
    private boolean m_2RightBumper_AlreadyPressed = false;
    private boolean m_2A_AlreadyPressed = false;
    private boolean m_2B_AlreadyPressed = false;
    private boolean m_2X_AlreadyPressed = false;
    private boolean m_2Y_AlreadyPressed = false;

    private Gamepad.RumbleEffect m_doubleRumble;

    private void initializeVera() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        m_vera.init(hardwareMap, false, false,
                VeraPipelineType.SIGNAL, telemetry);
        m_vera.setAllianceFromPriorAutonomousRun();

        m_vera.drivetrain.initMotorsToBrake();

        m_doubleRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 300)
                .addStep(0.0, 0.0, 100)
                .addStep(1.0, 1.0, 300)
                .build();

        if (Vera.isVisionTestMode) {
            telemetry.addData("WARNING:", "vision test mode!");
        } else {
            telemetry.addData("Status", "Initialized - " + m_vera.alliance);
        }
        telemetry.update();
    }

    private void getInputsFromSticksAndTriggers() {
        // Read gamepad1 sticks and pass commands to the drivetrainOld. Translates (mixes) Pitch,
        // Yaw, Roll, & Thrust commands from gamepad Sticks into Motor inputs.
        m_vera.drivetrain.translateSticksDroneFlightControls(
                -gamepad1.right_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                gamepad1.left_stick_y);

        m_vera.intake.commandDriverConeIntake(gamepad1.left_trigger, gamepad1.right_trigger);

        m_vera.lift.driverPlaceConeCommand(-gamepad2.right_stick_y);

        m_vera.intake.hasConeOverride(gamepad2.left_stick_y);
    }

    private void getCommandsFromG1Buttons() {
        if (gamepad1.left_bumper && !m_1LeftBumper_AlreadyPressed) {
            m_vera.lift.dropCone();
        } else if (gamepad1.right_bumper && !m_1RightBumper_AlreadyPressed) {
            m_vera.intake.moveToIntakeConePos(1);
        } else if (gamepad1.b && !m_1B_AlreadyPressed) {
            if (m_vera.intake.toggleLowJunctionMode()) {
                gamepad1.runRumbleEffect(m_doubleRumble);
            } else {
                gamepad1.rumble(300);
            }
        } else if (gamepad1.a && !m_1A_AlreadyPressed) {
            m_vera.intake.moveToIntakeConePos(5);
        } else if (gamepad1.x && !m_1X_AlreadyPressed) {
            m_vera.lift.moveLiftToHighPole();
        } else if (gamepad1.y && !m_1Y_AlreadyPressed) {
            m_vera.lift.moveLiftToMidPole();
        } else if (gamepad1.dpad_up && !m_1DpadUp_AlreadyPressed) {
            m_vera.intake.moveToIdlePos();
        } else if (gamepad1.dpad_right && !m_1DpadRight_AlreadyPressed) {
            m_vera.intake.moveToBeaconPlacePos();
        }

        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            m_vera.intake.setIntakeOverrideSpeed(-0.75);
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            m_vera.intake.setIntakeOverrideSpeed(0.75);
        } else {
            m_vera.intake.setIntakeOverrideSpeed(0.0);
        }

        m_1DpadUp_AlreadyPressed = gamepad1.dpad_up;
        m_1LeftBumper_AlreadyPressed = gamepad1.left_bumper;
        m_1RightBumper_AlreadyPressed = gamepad1.right_bumper;
        m_1A_AlreadyPressed = gamepad1.a;
        m_1B_AlreadyPressed = gamepad1.b;
        m_1X_AlreadyPressed = gamepad1.x;
        m_1Y_AlreadyPressed = gamepad1.y;
    }

    private void getCommandsFromG2Buttons() {
        if (gamepad2.left_bumper && !m_2LeftBumper_AlreadyPressed) {
            m_vera.lift.dropCone();
        } else if (gamepad2.right_bumper && !m_2RightBumper_AlreadyPressed) {
            m_vera.intake.moveToIntakeConePos(1);
        } else if (gamepad2.a && !m_2A_AlreadyPressed) {
            m_vera.lift.moveLiftToBottom();
        } else if (gamepad2.b && !m_2B_AlreadyPressed) {
            m_vera.lift.moveLiftToLowPole();
        } else if (gamepad2.x && !m_2X_AlreadyPressed) {
            m_vera.lift.moveLiftToHighPole();
        } else if (gamepad2.y && !m_2Y_AlreadyPressed) {
            m_vera.lift.moveLiftToMidPole();
        } else if (gamepad2.dpad_right && !m_2DpadRight_AlreadyPressed) {
            m_vera.lift.toggleClawOverride();
        } else if (gamepad2.dpad_up && !m_2DpadUp_AlreadyPressed) {
            m_vera.intake.toggleBeaconMode();
        }
        // NOTE: dpad_left and dpad_right are used in getCommandsFromG1Buttons to prevent
        // duplicated buttons from overwriting one another.

        m_2LeftBumper_AlreadyPressed = gamepad2.left_bumper;
        m_2RightBumper_AlreadyPressed = gamepad2.right_bumper;
        m_2A_AlreadyPressed = gamepad2.a;
        m_2B_AlreadyPressed = gamepad2.b;
        m_2X_AlreadyPressed = gamepad2.x;
        m_2Y_AlreadyPressed = gamepad2.y;
        m_2DpadUp_AlreadyPressed = gamepad2.dpad_up;
        m_2DpadRight_AlreadyPressed = gamepad2.dpad_right;
    }

    private void getCommandsFromButtons() {
        getCommandsFromG1Buttons();
        getCommandsFromG2Buttons();
    }

    private void getInputs() {
        // getInputs MUST be the first thing called in this function.
        m_vera.getInputs(false);
        getInputsFromSticksAndTriggers();
        getCommandsFromButtons();
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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            getInputs();  // getInputs MUST be called at the beginning of the main loop.
            commandVera();
            reportData();
        }
        stopVera();
    }
}
