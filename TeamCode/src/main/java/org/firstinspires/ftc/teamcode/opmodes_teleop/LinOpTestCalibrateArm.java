package org.firstinspires.ftc.teamcode.opmodes_teleop;

import static org.firstinspires.ftc.teamcode.middleware.CONSTANTS.df3;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="Test Calibrate Arm")
//@Disabled
public class LinOpTestCalibrateArm extends LinearOpMode {
    private enum ArmMode {
        INIT,
        NO_BRAKE,
        TEST_RUN_TO_POSITION
    }

    private final double ARM_MIN_DEG = -33;
    private final double ARM_TICK_PER_DEG = 6.69;
    private final int ARM_RANGE_TICK = 938;

    private final double ARM_DEG_PER_TICK = 1.0 / ARM_TICK_PER_DEG;
    private final double POS1_DEG = 0.0;
    private final double POS2_DEG = 90.0;
    private final int ARM_SPEED_INC = 500;
    private final int ARM_MAX_SPEED = 4500;
    private final int ARM_MIN_SPEED = 3000;

    private String CSV_LOG_PATH;
    private StringBuilder m_csvLogString = new StringBuilder();
    private DcMotorEx m_armMotor;
    private DcMotorEx m_wristMotor;
    private DigitalChannel m_armLimitSwitch;
    private ArmMode m_armMode = ArmMode.INIT;
    private PIDFCoefficients m_pidf;
    private int m_armSpeed = 500;
    private double m_armPos_deg;
    private int m_armPos_tick;
    private boolean m_1DpadUp_AlreadyPressed = false;
    private boolean m_1DpadDown_AlreadyPressed = false;
    private boolean m_1DpadLeft_AlreadyPressed = false;
    private boolean m_1DpadRight_AlreadyPressed = false;
    private boolean m_1A_AlreadyPressed = false;
    private boolean m_1B_AlreadyPressed = false;

    private void logCsvString(String record) {
        m_csvLogString.append(record).append("\n");
    }

    private void writeCsvLogFile() {
        if (m_csvLogString.length() > 0) {
            // The "false" argument indicates "do not append". We want a new file each time.
            try (FileWriter csvWriter = new FileWriter(CSV_LOG_PATH, false)) {
                csvWriter.write(m_csvLogString.toString());
            } catch (IOException e) {
                // do nothing
            }
        }
    }

    // This function must be called prior to calling armRunToPosition.
    private void resetArmMotor() {
        m_armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_armMotor.setPower(0.0);
        m_armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void moveArmToResetPos() {
        if (m_armLimitSwitch.getState()) {
            resetArmMotor();
            m_armMode = ArmMode.NO_BRAKE;
        } else {
            // Run arm toward limit switch until it is triggered.
            int currentPos_tick = m_armMotor.getCurrentPosition();
            m_armMotor.setTargetPosition(currentPos_tick - 10);
            m_armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m_armMotor.setVelocity(ARM_MIN_SPEED);
        }
    }

    private void initializeNoBrakeMode() {
        m_armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telemetry.addData("Initialized", "NO-BRAKE");
        telemetry.addData("pidf", m_pidf.toString());
        telemetry.update();
    }

    private void initializeRunToPositionMode() {
        m_armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Initialized", "RUN-TO-POSITION");
        telemetry.addData("pidf", m_pidf.toString());
        telemetry.update();
    }

    private void initialize() {
        String extStoragePath = Environment.getExternalStorageDirectory().getAbsolutePath();
        CSV_LOG_PATH = String.format("%s/FIRST/data/calibrate.csv", extStoragePath);

        // The wrist motor just needs to BRAKE at its startup position.
        m_wristMotor = hardwareMap.get(DcMotorEx.class, "WristMotor");
        m_wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m_armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        m_armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        m_armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_pidf = m_armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        logCsvString("pidf" +
                ", " + m_pidf.p +
                ", " + m_pidf.i +
                ", " + m_pidf.d +
                ", " + m_pidf.f);
        m_pidf.p = 11 * 0.8; //Uses Ziegler–Nichols method for PD tuning
        m_pidf.i = 0;
        m_pidf.d = 11 * 0.1;
        m_pidf.f = 0;
        m_armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, m_pidf);

        m_armLimitSwitch = hardwareMap.get(DigitalChannel.class,"ArmLimitSwitch");
        m_armLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        getHardwareInputs();
        initializeNoBrakeMode();
    }

    public void armRunToPosition(double pos_deg, int speed) {
        int pos_tick = (int)((pos_deg - ARM_MIN_DEG) * ARM_TICK_PER_DEG);
        m_armMotor.setTargetPosition(pos_tick);
        m_armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_armMotor.setVelocity(speed);
    }

    private void getMoveArmInputs() {
        if (gamepad1.dpad_left && !m_1DpadLeft_AlreadyPressed) {
            armRunToPosition(POS1_DEG, m_armSpeed);
        } else if (gamepad1.dpad_right && !m_1DpadRight_AlreadyPressed) {
            armRunToPosition(POS2_DEG, m_armSpeed);
        } else if (gamepad1.dpad_up && !m_1DpadUp_AlreadyPressed) {
            m_armSpeed = Math.min(ARM_MAX_SPEED, m_armSpeed + ARM_SPEED_INC);
        } else if (gamepad1.dpad_down && !m_1DpadDown_AlreadyPressed) {
            m_armSpeed = Math.max(ARM_MIN_SPEED, m_armSpeed - ARM_SPEED_INC);
        }

        m_1DpadLeft_AlreadyPressed = gamepad1.dpad_left;
        m_1DpadRight_AlreadyPressed = gamepad1.dpad_right;
        m_1DpadUp_AlreadyPressed = gamepad1.dpad_up;
        m_1DpadDown_AlreadyPressed = gamepad1.dpad_down;
    }

    private void computeArmValues() {
        m_armPos_deg = ARM_MIN_DEG + (m_armPos_tick * ARM_DEG_PER_TICK);
    }

    private void getHardwareInputs() {
        m_armPos_tick = m_armMotor.getCurrentPosition();
        computeArmValues();
    }

    private void getInputs() {
        getHardwareInputs();

        if (gamepad1.a && !m_1A_AlreadyPressed) {
            m_armMode = ArmMode.NO_BRAKE;
            initializeNoBrakeMode();
            computeArmValues();
        } else if (gamepad1.b && !m_1B_AlreadyPressed) {
            m_armMode = ArmMode.TEST_RUN_TO_POSITION;
            initializeRunToPositionMode();
            computeArmValues();
        }

        m_1A_AlreadyPressed = gamepad1.a;
        m_1B_AlreadyPressed = gamepad1.b;

        if (m_armMode == ArmMode.TEST_RUN_TO_POSITION) {
            getMoveArmInputs();
        }
    }

    private void reportData() {
        telemetry.addData("mode", m_armMode);
        telemetry.addData("speed", m_armSpeed);
        telemetry.addData("raw Pos Tick", m_armPos_tick);
        telemetry.addData("pos deg", df3.format(m_armPos_deg));
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            switch (m_armMode) {
                case INIT:
                    moveArmToResetPos();
                    break;
                default:
                    getInputs();
                    reportData();
                    break;
            }
        }

        writeCsvLogFile();
    }
}

