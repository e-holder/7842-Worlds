package org.firstinspires.ftc.teamcode.opmodes_teleop;

import static org.firstinspires.ftc.teamcode.middleware.CONSTANTS.df3;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="Test Calibrate Wrist")
//@Disabled
public class LinOpTestCalibrateWrist extends LinearOpMode {

    private final double WRIST_MIN_DEG = 0.0;
    private final double WRIST_MAX_DEG = 290.0; // TODO: Get new calibration for this number.
    private final double WRIST_MIN_DEG_SERVO = 0.19;  // We can command the servo below this.
    private final double WRIST_MAX_DEG_SERVO = 1.0;

    private final double WRIST_SERVO_PER_DEG = (WRIST_MAX_DEG_SERVO - WRIST_MIN_DEG_SERVO) /
            (WRIST_MAX_DEG - WRIST_MIN_DEG);
    private final double WRIST_DEG_PER_SERVO = 1.0 / WRIST_SERVO_PER_DEG;
    private final double POS1_DEG = 0.0;
    private final double POS2_DEG = 180.0;

    private String CSV_LOG_PATH;
    private StringBuilder m_csvLogString = new StringBuilder();
    private Servo m_wristServo;
    private double m_wristPos_deg;
    private double m_wristServoPos;;
    private boolean m_1DpadUp_AlreadyPressed = false;
    private boolean m_1DpadDown_AlreadyPressed = false;
    private boolean m_1DpadLeft_AlreadyPressed = false;
    private boolean m_1DpadRight_AlreadyPressed = false;

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

    private void initialize() {
        String extStoragePath = Environment.getExternalStorageDirectory().getAbsolutePath();
        CSV_LOG_PATH = String.format("%s/FIRST/data/calibrate.csv", extStoragePath);

        m_wristServo = hardwareMap.get(Servo.class, "WristServo");
        m_wristServo.setDirection(Servo.Direction.REVERSE);

        getHardwareInputs();
    }

    public void wristMoveToPosition(double pos_deg) {
        double servoPos = WRIST_MIN_DEG_SERVO + (pos_deg * WRIST_SERVO_PER_DEG);
        m_wristServo.setPosition(servoPos);
    }

    private void getHardwareInputs() {
        m_wristServoPos = m_wristServo.getPosition();
        m_wristPos_deg = WRIST_MIN_DEG +
                ((m_wristServoPos - WRIST_MIN_DEG_SERVO) * WRIST_DEG_PER_SERVO);
    }

    private void getInputs() {
        getHardwareInputs();

        if (gamepad1.dpad_left && !m_1DpadLeft_AlreadyPressed) {
            wristMoveToPosition(POS1_DEG);
        } else if (gamepad1.dpad_right && !m_1DpadRight_AlreadyPressed) {
            wristMoveToPosition(POS2_DEG);
        } else if (gamepad1.dpad_up && !m_1DpadUp_AlreadyPressed) {
            double servoPos = Math.min(1.0, m_wristServoPos + 0.01);
            m_wristServo.setPosition(servoPos);
        } else if (gamepad1.dpad_down && !m_1DpadDown_AlreadyPressed) {
            double servoPos = Math.max(0.0, m_wristServoPos - 0.01);
            m_wristServo.setPosition(servoPos);
        }

        m_1DpadLeft_AlreadyPressed = gamepad1.dpad_left;
        m_1DpadRight_AlreadyPressed = gamepad1.dpad_right;
        m_1DpadUp_AlreadyPressed = gamepad1.dpad_up;
        m_1DpadDown_AlreadyPressed = gamepad1.dpad_down;
    }

    private void reportData() {
        telemetry.addData("servo Pos", df3.format(m_wristServoPos));
        telemetry.addData("wrist Deg", df3.format(m_wristPos_deg));
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            getInputs();
            reportData();
        }

        writeCsvLogFile();
    }
}

