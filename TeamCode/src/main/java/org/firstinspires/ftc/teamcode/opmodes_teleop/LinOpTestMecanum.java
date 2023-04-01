package org.firstinspires.ftc.teamcode.opmodes_teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Mecanum Test")
@Disabled
public class LinOpTestMecanum extends LinearOpMode {
    public enum StickTranslateStyle {
        TRADITIONAL,           // Select with Y button (default)
        TRADITIONAL_RL_YAW,    // Select with X button
        DRONE,                 // Select with A button
        TANK                   // Select with B button
    }

    private DcMotor m_motorFL = null;
    private DcMotor m_motorBL = null;
    private DcMotor m_motorFR = null;
    private DcMotor m_motorBR = null;
    private double m_fr = 0, m_fl = 0, m_br = 0, m_bl = 0; // Motor power: Front/Back, Left/Right
    private double m_pitch, m_yaw, m_roll; // Commands
    private StickTranslateStyle m_stickTranslateStyle = StickTranslateStyle.TRADITIONAL;

    private void initializeHardware() {
        m_motorFL = hardwareMap.get(DcMotor.class, "FL");  // Front Left motor
        m_motorBL = hardwareMap.get(DcMotor.class, "BL");  // Back Left motor
        m_motorFR = hardwareMap.get(DcMotor.class, "FR");  // Front Right motor
        m_motorBR = hardwareMap.get(DcMotor.class, "BR");  // Back Right motor

        m_motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        m_motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        m_motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        m_motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        m_motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m_motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void translateSticksTraditional() {
        m_pitch = -gamepad1.left_stick_y;
        m_yaw   =  gamepad1.right_stick_y;
        m_roll  =  gamepad1.left_stick_x;
        m_fr = m_pitch - m_yaw - m_roll;
        m_fl = m_pitch + m_yaw + m_roll;
        m_br = m_pitch - m_yaw + m_roll;
        m_bl = m_pitch + m_yaw - m_roll;
    }

    private void translateSticksTraditionalRLYaw() {
        m_pitch = -gamepad1.left_stick_y;
        m_yaw   =  gamepad1.right_stick_x;  // X instead of Y (versus Traditional)
        m_roll  =  gamepad1.left_stick_x;
        m_fr = m_pitch - m_yaw - m_roll;
        m_fl = m_pitch + m_yaw + m_roll;
        m_br = m_pitch - m_yaw + m_roll;
        m_bl = m_pitch + m_yaw - m_roll;
    }

    private void translateSticksDroneFlightControls() {
        m_pitch  = -gamepad1.right_stick_y;
        m_yaw    =  gamepad1.left_stick_x;
        m_roll   =  gamepad1.right_stick_x;
        double thrust = gamepad1.left_stick_y;
        m_fr = m_pitch - m_yaw - m_roll;
        m_fl = m_pitch + m_yaw + m_roll;
        m_br = m_pitch - m_yaw + m_roll;
        m_bl = m_pitch + m_yaw - m_roll;

        final double slowThresh = -0.5;
        final double slowFactor = 0.65;
        if (thrust < slowThresh) { // Incorporate Thrust (slow down)
            m_fr *= slowFactor;
            m_fl *= slowFactor;
            m_br *= slowFactor;
            m_bl *= slowFactor;
        }
    }

    private void translateSticksTankControls() {
        double stickYL = -gamepad1.left_stick_y;
        double stickYR = -gamepad1.right_stick_y;
        double stickXL = -gamepad1.left_stick_x;
        double stickXR = -gamepad1.right_stick_x;
        m_fr = stickYR + stickXR + stickXL;
        m_fl = stickYL - stickXR - stickXL;
        m_br = stickYR - stickXR - stickXL;
        m_bl = stickYL + stickXR + stickXL;
    }

    // Translates (mixes)  Pitch, Yaw, and Roll commands from gamepad Sticks into Motor inputs
    // based on user preference.
    private void getMotorInputsFromSticks() {
        // If driver has pressed a button, change style.
        if (gamepad1.y) {
            // Y button = Traditional
            m_stickTranslateStyle = StickTranslateStyle.TRADITIONAL;
        }
        else if (gamepad1.x) {
            // X button = Drone Flight Controls
            m_stickTranslateStyle = StickTranslateStyle.DRONE;
        }
        else if (gamepad1.a) {
            // A button = Traditional + R/L Yaw
            m_stickTranslateStyle = StickTranslateStyle.TRADITIONAL_RL_YAW;
        }
        else if (gamepad1.b) {
            // B button = Tank Controls
            m_stickTranslateStyle = StickTranslateStyle.TANK;
        }


        switch (m_stickTranslateStyle) {
            case TRADITIONAL:
                translateSticksTraditional();
                break;
            case DRONE:
                translateSticksDroneFlightControls();
                break;
            case TRADITIONAL_RL_YAW:
                translateSticksTraditionalRLYaw();
                break;
            case TANK:
                translateSticksTankControls();
                break;
        }

    }

    // Scales motor power such that max power is never larger in magnitude than 1.
    private void scaleMotorPower() {
        double cur_max = Math.max(
                Math.max(Math.abs(m_fr), Math.abs(m_fl)),
                Math.max(Math.abs(m_br), Math.abs(m_bl)));
        if (cur_max > 1) {
            m_fr /= cur_max;
            m_fl /= cur_max;
            m_br /= cur_max;
            m_bl /= cur_max;
        }
    }

    private void commandMotors() {
        scaleMotorPower();  // Ensure max power <= 1

        m_motorFL.setPower(m_fl);
        m_motorFR.setPower(m_fr);
        m_motorBL.setPower(m_bl);
        m_motorBR.setPower(m_br);
    }

    private void updateTelemetry() {
        switch (m_stickTranslateStyle) {
            case TRADITIONAL:
                telemetry.addLine("StickTranslateStyle: Traditional");
                break;
            case TRADITIONAL_RL_YAW:
                telemetry.addLine("StickTranslateStyle: Traditional + R/L Yaw");
                break;
            case DRONE:
                telemetry.addLine("StickTranslateStyle: Drone Flight Controls");
                break;
            case TANK:
                telemetry.addLine("StickTranslateStyle: Tank Controls");
                break;
        }
        telemetry.addData("FL Velocity", m_motorFL.getPower());
        telemetry.addData("BL Velocity", m_motorBL.getPower());
        telemetry.addData("BR Velocity", m_motorBR.getPower());
        telemetry.addData("FR Velocity", m_motorFR.getPower());
        telemetry.addData("Stick Style", m_stickTranslateStyle);
        telemetry.addData("DID YOU REMEMBER TO SPRAY THE FIELD?", "Hi");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        initializeHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            getMotorInputsFromSticks();
            commandMotors();
            updateTelemetry();
        }
    }
}

