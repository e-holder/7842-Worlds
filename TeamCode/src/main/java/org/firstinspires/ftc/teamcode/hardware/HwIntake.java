package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class HwIntake {

    // The TICKS_PER_DEG variables are not ideal, in that they are not computed from the motor's
    // hardware interface. Ideally we should be calling <motor>.getMotorType().getTicksPerRev().
    // Theoretically the code would then work if we changed out motors with different specs.
    private final double ARM_TICKS_PER_DEG = 6.69;
    private final double ARM_DEG_PER_TICK = 1.0 / ARM_TICKS_PER_DEG;
    public final double MIN_ARM_DEG = -33.0;

    private final double WRIST_MIN_DEG = 0.0;
    private final double WRIST_MAX_DEG = 290.0;
    private final double WRIST_MIN_DEG_SERVO = 0.07;  // We can command the servo below this.
    private final double WRIST_MAX_DEG_SERVO = 1.0;
    private final double WRIST_SERVO_PER_DEG = (WRIST_MAX_DEG_SERVO - WRIST_MIN_DEG_SERVO) /
            (WRIST_MAX_DEG - WRIST_MIN_DEG);
    private final double WRIST_DEG_PER_SERVO = 1.0 / WRIST_SERVO_PER_DEG;

    private double LEFT_TAPE_SENSOR_RED_SCALE = 1.0;
    private double LEFT_TAPE_SENSOR_BLUE_SCALE = 0.88;
    private double RIGHT_TAPE_SENSOR_RED_SCALE = 0.92;
    private double RIGHT_TAPE_SENSOR_BLUE_SCALE = 1.0;

    private DcMotorEx m_armMotor;
    private DcMotorEx m_intakeWheelMotor;
    private Servo m_wristServo;
    private DigitalChannel m_armLimitSwitch;
    private ColorSensor m_tapeSensorL;
    private ColorSensor m_tapeSensorR;
    private PIDFCoefficients m_pidf;

    public void init(HardwareMap hwMap) {
        m_armMotor = hwMap.get(DcMotorEx.class, "ArmMotor");
        m_armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        m_armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_pidf = m_armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        m_pidf.p = 11 * 0.8; // Ziegler–Nichols method for PD tuning
        m_pidf.i = 0;
        m_pidf.d = 11 * 0.1;
        m_pidf.f = 0;
        m_armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, m_pidf);
        m_armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m_wristServo = hwMap.get(Servo.class, "WristServo");
        m_wristServo.setDirection(Servo.Direction.REVERSE);

        m_intakeWheelMotor = hwMap.get(DcMotorEx.class, "IntakeMotor");
        m_intakeWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        m_intakeWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_intakeWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m_armLimitSwitch = hwMap.get(DigitalChannel.class,"ArmLimitSwitch");
        m_armLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        m_tapeSensorL = hwMap.get(ColorSensor.class, "LeftTapeSensor");  // Control Hub I2C Bus 2
        m_tapeSensorR = hwMap.get(ColorSensor.class, "RightTapeSensor"); // Control Hub I2C Bus 1
    }

    // =========  Arm methods ==============

    public boolean isLimitSwitchPressed() {
        return m_armLimitSwitch.getState();
    }

    public double getIntakeArmMotorCurrent_amp() {
        return m_armMotor.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isArmBusy() {
        // Used for RUN_TO_POSITION mode.
        return m_armMotor.isBusy();
    }

    public void resetArmMotor(int currentPos_tick) {
        m_armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_armMotor.setPower(0.0);
        m_armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_armMotor.setTargetPosition(currentPos_tick + 1);
    }

    // Note: The position is in degrees from the vertical position (negative is toward the lift).
    public void armRunToPosition(double pos_deg, double speed) {
        int ticks = (int)((pos_deg - MIN_ARM_DEG) * ARM_TICKS_PER_DEG);
        m_armMotor.setTargetPosition(ticks);
        m_armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_armMotor.setVelocity(speed);
    }

    public int getArmPosition_ticks() {
        return m_armMotor.getCurrentPosition();
    }

    public double getArmPosition_deg() {
        return (m_armMotor.getCurrentPosition() * ARM_DEG_PER_TICK) + MIN_ARM_DEG;
    }

    // =========  Wrist methods ==============

    public double getWristServoPos() {
        return m_wristServo.getPosition();
    }

    public double getWristPos_deg(double wristServoPos) {
        return WRIST_MIN_DEG + ((wristServoPos - WRIST_MIN_DEG_SERVO) * WRIST_DEG_PER_SERVO);
    }

    public void wristMoveToPosition(double pos_deg) {
        double servoPos = WRIST_MIN_DEG_SERVO + (pos_deg * WRIST_SERVO_PER_DEG);
        servoPos = Math.min(1.0, servoPos);
        servoPos = Math.max(0.0, servoPos);
        m_wristServo.setPosition(servoPos);
    }

    // =========  Intake Wheel methods ==============

    public double getIntakeWheelMotorCurrent_amp() {
        return m_intakeWheelMotor.getCurrent(CurrentUnit.AMPS);
    }

    public void setIntakeWheelSpeed(double intakeSpeed) {
        // Simple pass-through of speed to power.
        m_intakeWheelMotor.setPower(intakeSpeed);
    }

    // =========  Red/Blue Tape Detection methods ==============

    public int getLeftTapeSensorRed() {
        return (int)(m_tapeSensorL.red() * LEFT_TAPE_SENSOR_RED_SCALE);
    }

    public int getLeftTapeSensorBlue() {
        return (int)(m_tapeSensorL.blue() * LEFT_TAPE_SENSOR_BLUE_SCALE);
    }

    public int getRightTapeSensorRed() {
        return (int)(m_tapeSensorR.red() * RIGHT_TAPE_SENSOR_RED_SCALE);
    }

    public int getRightTapeSensorBlue() {
        return (int)(m_tapeSensorR.blue() * RIGHT_TAPE_SENSOR_BLUE_SCALE);
    }
}
