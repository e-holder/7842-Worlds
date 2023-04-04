package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.ml.distance.DistanceMeasure;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HwLift {

    private final double LIFT_MAX_TICKS = 1207.0;
    public final double LIFT_MAX_IN = 45.5;

    // These two variables are not ideal, in that they are not computed from the motor's hardware
    // interface. Ideally we should be calling <motor>.getMotorType().getTicksPerRev().
    // Theoretically the code would then work if we changed out motors with different specs.
    private final double LIFT_TICKS_PER_IN = LIFT_MAX_TICKS / LIFT_MAX_IN;
    private final double LIFT_INCHES_PER_TICK = LIFT_MAX_IN / LIFT_MAX_TICKS;

    private DcMotorEx m_liftMotor = null;
    private DigitalChannel m_limitSwitch = null;
    private Servo m_liftClaw = null;
    private DistanceSensor m_middleManDistanceSensor;
    private PIDFCoefficients m_pidf;

    public void init(HardwareMap hwMap) {
        m_liftMotor = hwMap.get(DcMotorEx.class, "LiftMotor"); // Expansion Hub port 0.
        m_liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        m_liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_pidf = m_liftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        m_pidf.p = 38 * 0.8;   // Ziegler–Nichols method for PD tuning
        m_pidf.i = 0;
        m_pidf.d = 38 * 0.1 * 0.1;
        m_pidf.f = 0;
        m_liftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, m_pidf);
        m_liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m_limitSwitch = hwMap.get(DigitalChannel.class, "LiftLimitSwitch");
        m_limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        m_liftClaw = hwMap.get(Servo.class, "LiftClawServo");

        m_middleManDistanceSensor = hwMap.get(DistanceSensor.class, "MiddlemanDistanceSensor");
    }

    public boolean isLiftBusy () {
        return m_liftMotor.isBusy();
    }

    public void resetLiftMotor(int currentPos_tick) {
        m_liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_liftMotor.setPower(0.0);
        m_liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_liftMotor.setTargetPosition(currentPos_tick + 1);
    }

    public int liftRunToPosition(double pos_in, double speed_tps) {
        int ticks = (int) (pos_in * LIFT_TICKS_PER_IN);
        m_liftMotor.setTargetPosition(ticks);
        m_liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_liftMotor.setVelocity(speed_tps);
        return ticks;
    }

    public int getLiftPosition_ticks() {
        return m_liftMotor.getCurrentPosition();
    }

    public double getLiftPosition_in() {
        return m_liftMotor.getCurrentPosition() * LIFT_INCHES_PER_TICK;
    }

    public double getLiftMotorCurrent_amp() {
        return m_liftMotor.getCurrent(CurrentUnit.AMPS);
    }

    public double getLiftMotorPower() {
        return m_liftMotor.getPower();
    }

    public boolean isLimitSwitchPressed() {
        return m_limitSwitch.getState();
    }

    // ========== Claw Servo ===================

    public void setLiftClawPos(double position) {
        m_liftClaw.setPosition(position);
    }

    public double getLiftClawPos() {
        return m_liftClaw.getPosition();
    }

    // ========== Middleman Cone Sensor ==============

    public double getMiddlemanSensorDistance_in() {
        return m_middleManDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    // ===============================================

    private void useSomeVariablesToSatisfyAndroidStudio() {
        double x = LIFT_TICKS_PER_IN + LIFT_INCHES_PER_TICK;
    }
}
