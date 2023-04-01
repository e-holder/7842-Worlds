package org.firstinspires.ftc.teamcode.middleware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HwDrivetrain;

public class Drivetrain implements CONSTANTS {

    // CONSTANTS ==================================================================================

    // Adjustment constants for low-thrust control in drone mode.
    private static final double DRONE_CONTROLS_SLOW_THRESH = -0.5;
    private static final double DRONE_CONTROLS_SLOW_FACTOR = 0.65;

    // Factor to reduce overall sensitivity in teleop.
    // Added for 2022 KY scrimmage 1 (0.9 factor)
    // Reduced for new drivers in 2022 KY scrimmage 2 (0.75 factor)
    private static final double TELEOP_POWER_FACTOR = 0.9;

    // MEMBER DATA ================================================================================
    // SUBSYSTEM has an instance of its corresponding hardware class here.
    private HwDrivetrain m_hwDrivetrain;

    private Vera m_vera;
    private StringBuilder m_csvLogStr = new StringBuilder();

    // Variables that are commands to the hardware layer.
    // Commanded motor power: Front/Back, Left/Right
    private double m_fl = 0, m_fr = 0, m_bl = 0, m_br = 0;

    // SUBSYSTEM has a public constructor here.
    public Drivetrain(Vera vera) {
        m_vera = vera;
        // SUBSYSTEM constructs its corresponding hardware class instance here.
        m_hwDrivetrain = vera.getHwDrivetrain();
    }

    // SUBSYSTEM has a private constructor here.
    private Drivetrain() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Default constructor not supported.");
    }

    public void logCsvString(String record) { m_csvLogStr.append(record).append("\n"); }

    public StringBuilder getLogString() {
        return m_csvLogStr;
    }

    public void translateSticksDroneFlightControls(double pitch, double yaw,
                                                   double roll, double thrust) {
        m_fl = (pitch + yaw + roll) * TELEOP_POWER_FACTOR;
        m_fr = (pitch - yaw - roll) * TELEOP_POWER_FACTOR;
        m_bl = (pitch + yaw - roll) * TELEOP_POWER_FACTOR;
        m_br = (pitch - yaw + roll) * TELEOP_POWER_FACTOR;

        if (thrust < DRONE_CONTROLS_SLOW_THRESH) { // Incorporate Thrust (slow down)
            m_fl *= DRONE_CONTROLS_SLOW_FACTOR;
            m_fr *= DRONE_CONTROLS_SLOW_FACTOR;
            m_bl *= DRONE_CONTROLS_SLOW_FACTOR;
            m_br *= DRONE_CONTROLS_SLOW_FACTOR;
        }
    }

    public void reportData(Telemetry telemetry) {
//        if (false) {
//            // Add CSV logging and/or telemetry here
//            logCsvString("remDist, " + df3.format(m_remainingDist_in));
//        }
    }
}
