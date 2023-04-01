package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HwDrivetrain {
    private DcMotorEx m_motorFL = null;
    private DcMotorEx m_motorFR = null;
    private DcMotorEx m_motorBL = null;
    private DcMotorEx m_motorBR = null;

    public void init(HardwareMap hwMap) {
        m_motorFL = hwMap.get(DcMotorEx.class, "FL");  // Front Left motor
        m_motorFR = hwMap.get(DcMotorEx.class, "FR");  // Front Right motor
        m_motorBL = hwMap.get(DcMotorEx.class, "BL");  // Back Left motor
        m_motorBR = hwMap.get(DcMotorEx.class, "BR");  // Back Right motor

        // We want positive power to move all 4 wheels in the forward direction. Since two motors
        // are mounted 180 degrees from the other two, we need to reverse their operation.
        m_motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        m_motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        m_motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        m_motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        m_motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m_motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
