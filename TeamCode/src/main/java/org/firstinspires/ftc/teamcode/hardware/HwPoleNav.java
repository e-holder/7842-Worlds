package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HwPoleNav {
    private DistanceSensor m_poleSensorL;
    private DistanceSensor m_poleSensorR;

    public void init(HardwareMap hwMap) {
        m_poleSensorL = hwMap.get(DistanceSensor.class, "PoleSensorL");
        m_poleSensorR = hwMap.get(DistanceSensor.class, "PoleSensorR");
    }

    public double getDistanceR_in() {
        // 2M distance sensor
        return m_poleSensorR.getDistance(DistanceUnit.INCH);
    }

    public double getDistanceL_in() {
        // 2M distance sensor
        return m_poleSensorL.getDistance(DistanceUnit.INCH);

    }
}