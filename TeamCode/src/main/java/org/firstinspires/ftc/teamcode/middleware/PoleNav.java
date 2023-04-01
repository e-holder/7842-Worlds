package org.firstinspires.ftc.teamcode.middleware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HwPoleNav;

public class PoleNav implements CONSTANTS {

    // MEMBER DATA ================================================================================

    // Delta to apply to measurements due to distance sensors not being even with sides of robot.
    private final double LEFT_SENSOR_DELTA_ROBOT_SIDE_IN = -0.125;
    private final double RIGHT_SENSOR_DELTA_ROBOT_SIDE_IN = 0.0;

    // SUBSYSTEM has an instance of its corresponding hardware class here.
    private HwPoleNav m_hwPoleNav;

    private double m_distanceRight_in;
    private double m_distanceLeft_in;

    private Vera m_vera;
    private StringBuilder m_csvLogStr = new StringBuilder();

    // SUBSYSTEM has a public constructor here.
    public PoleNav(Vera vera) {
        m_vera = vera;
        // SUBSYSTEM constructs its corresponding hardware class instance here.
        m_hwPoleNav = vera.getHwPoleNav();
    }

    // SUBSYSTEM has a private constructor here.
    private PoleNav() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Default constructor not supported.");
    }

    public void getInputs(boolean isAutonomous) {
        if (isAutonomous) {
            m_distanceLeft_in = m_hwPoleNav.getDistanceL_in() + LEFT_SENSOR_DELTA_ROBOT_SIDE_IN;
            m_distanceRight_in = m_hwPoleNav.getDistanceR_in() + RIGHT_SENSOR_DELTA_ROBOT_SIDE_IN;
        }
    }

    public double getDistanceL_in() {
        return m_distanceLeft_in;
    }

    public double getDistanceR_in() {
        return m_distanceRight_in;
    }

    public void logCsvString(String record) {
        m_csvLogStr.append(record).append("\n");
    }

    public StringBuilder getLogString() {
        return m_csvLogStr;
    }

    public void reportData(Telemetry telemetry) {
        if (false) {
            logCsvString(
                    "distL, " + df3.format(m_distanceLeft_in) +
                            ", distR, " + df3.format(m_distanceRight_in));
        }
    }
}