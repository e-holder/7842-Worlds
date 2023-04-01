package org.firstinspires.ftc.teamcode.opmodes_teleop;

import static org.firstinspires.ftc.teamcode.middleware.CONSTANTS.df3;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="Test Calibrate Lift")
//@Disabled
public class LinOpTestCalibrateLift extends LinearOpMode {
    private enum LiftMode {
        INIT,
        NO_BRAKE,
        TEST_RUN_TO_POSITION
    }

    private final double LIFT_MAX_TICK = 1207.0;
    private final double LIFT_MAX_IN = 45.5;

    private final double LIFT_TICK_PER_IN = LIFT_MAX_TICK / LIFT_MAX_IN;
    private final double LIFT_INCH_PER_TICK = LIFT_MAX_IN / LIFT_MAX_TICK;
    private final double POS1_IN = 0.0;
    private final double POS2_IN = 35.0;
    private final int LIFT_SPEED_INC = 500;
    private final int LIFT_MAX_SPEED = 4500;
    private final int LIFT_MIN_SPEED = 50;

    private String CSV_LOG_PATH;
    private StringBuilder m_csvLogString = new StringBuilder();
    private DcMotorEx m_liftMotor;
    private LiftMode m_liftMode = LiftMode.INIT;
    private PIDFCoefficients m_pidf;
    private int m_liftSpeed = 500;
    private double m_liftPos_in;
    private int m_liftPos_tick;
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
    private void resetLiftMotor() {
        m_liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_liftMotor.setPower(0.0);
        m_liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void moveLiftToResetPos() {
        if (m_liftMotor.getCurrent(CurrentUnit.AMPS) >= 6.0) {
            resetLiftMotor();
            m_liftMode = LiftMode.NO_BRAKE;
        } else {
            // Run lift toward limit switch until it is triggered.
            int currentPos_tick = m_liftMotor.getCurrentPosition();
            m_liftMotor.setTargetPosition(currentPos_tick - 100);
            m_liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m_liftMotor.setVelocity(LIFT_MAX_SPEED);
        }
    }

    private void initializeNoBrakeMode() {
        m_liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telemetry.addData("Initialized", "NO-BRAKE");
        telemetry.addData("pidf", m_pidf.toString());
        telemetry.update();
    }

    private void initializeRunToPositionMode() {
        m_liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Initialized", "RUN-TO-POSITION");
        telemetry.addData("pidf", m_pidf.toString());
        telemetry.update();
    }

    private void initialize() {
        String extStoragePath = Environment.getExternalStorageDirectory().getAbsolutePath();
        CSV_LOG_PATH = String.format("%s/FIRST/data/calibrate.csv", extStoragePath);

        m_liftMotor = hardwareMap.get(DcMotorEx.class, "LiftMotor");
        m_liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        m_liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_pidf = m_liftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        logCsvString("pidf" +
                ", " + m_pidf.p +
                ", " + m_pidf.i +
                ", " + m_pidf.d +
                ", " + m_pidf.f);
        // TODO: Find default pidf values and plug those in here. Then calibrate these coefficients
        //  for better performance.
        m_pidf.p = 38 * 0.8;
        m_pidf.i = 0;
        m_pidf.d = 38 * 0.1 * 0.1;
        m_pidf.f = 0;
        m_liftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, m_pidf);

        getHardwareInputs();
        initializeNoBrakeMode();
    }

    public void liftRunToPosition(double pos_in, int speed) {
        int pos_tick = (int)(pos_in * LIFT_TICK_PER_IN);
        m_liftMotor.setTargetPosition(pos_tick);
        m_liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_liftMotor.setVelocity(speed);
    }

    private void getMoveLiftInputs() {
        if (gamepad1.dpad_left && !m_1DpadLeft_AlreadyPressed) {
            liftRunToPosition(POS1_IN, m_liftSpeed);
        } else if (gamepad1.dpad_right && !m_1DpadRight_AlreadyPressed) {
            liftRunToPosition(POS2_IN, m_liftSpeed);
        } else if (gamepad1.dpad_up && !m_1DpadUp_AlreadyPressed) {
            m_liftSpeed = Math.min(LIFT_MAX_SPEED, m_liftSpeed + LIFT_SPEED_INC);
        } else if (gamepad1.dpad_down && !m_1DpadDown_AlreadyPressed) {
            m_liftSpeed = Math.max(LIFT_MIN_SPEED, m_liftSpeed - LIFT_SPEED_INC);
        }

        m_1DpadLeft_AlreadyPressed = gamepad1.dpad_left;
        m_1DpadRight_AlreadyPressed = gamepad1.dpad_right;
        m_1DpadUp_AlreadyPressed = gamepad1.dpad_up;
        m_1DpadDown_AlreadyPressed = gamepad1.dpad_down;
    }

    private void computeLiftValues() {
        m_liftPos_in = m_liftPos_tick * LIFT_INCH_PER_TICK;
    }

    private void getHardwareInputs() {
        m_liftPos_tick = m_liftMotor.getCurrentPosition();
        computeLiftValues();
    }

    private void getInputs() {
        getHardwareInputs();

        if (gamepad1.a && !m_1A_AlreadyPressed) {
            m_liftMode = LiftMode.NO_BRAKE;
            initializeNoBrakeMode();
            computeLiftValues();
        } else if (gamepad1.b && !m_1B_AlreadyPressed) {
            m_liftMode = LiftMode.TEST_RUN_TO_POSITION;
            initializeRunToPositionMode();
            computeLiftValues();
        }

        m_1A_AlreadyPressed = gamepad1.a;
        m_1B_AlreadyPressed = gamepad1.b;

        if (m_liftMode == LiftMode.TEST_RUN_TO_POSITION) {
            getMoveLiftInputs();
        }
    }

    private void reportData() {
        telemetry.addData("mode", m_liftMode);
        telemetry.addData("speed", m_liftSpeed);
        telemetry.addData("pos Tick", m_liftPos_tick);
        telemetry.addData("pos Inch", df3.format(m_liftPos_in));
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            switch (m_liftMode) {
                case INIT:
                    moveLiftToResetPos();
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

