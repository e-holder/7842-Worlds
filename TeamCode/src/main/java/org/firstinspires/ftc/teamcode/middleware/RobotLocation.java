package org.firstinspires.ftc.teamcode.middleware;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Locale;

public class RobotLocation implements CONSTANTS {
    // The origin is the center of the field. Positive Y is away from the audience ("North").
    // Positive X is toward the red side of the field ("East").
    private final Position m_pos_in = new Position();
    // North (0 deg heading) is away from the audience. Positive headings are clockwise from North.
    private double m_fieldHeading_deg;
    // "North" for the IMU is whatever direction it was facing when the OpMode Initialized. We have
    // to continually offset our heading on the field to account for this.
    private final double m_imuHeadingOffset_deg;
    private double m_headingCorrection_deg;

    public RobotLocation(double robotStartingOrientation_deg, double x_in, double y_in) {
        m_imuHeadingOffset_deg = robotStartingOrientation_deg;
        m_fieldHeading_deg = m_imuHeadingOffset_deg;
        m_pos_in.x = x_in;
        m_pos_in.y = y_in;
    }

    private RobotLocation() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Default constructor not supported.");
    }

    // Gets the "field" heading of the robot.
    public double getHeading_deg() {
        double angle_deg = m_fieldHeading_deg + m_headingCorrection_deg;
        while (angle_deg > 180.0) {
            angle_deg -= 360.0;
        }
        while (angle_deg < -180.0) {
            angle_deg += 360.0;
        }
        return angle_deg;
    }

    // Returns a string representation of the robot location (heading and position) in field
    // coordinates.
    @Override
    public String toString() {
        return String.format(Locale.US,
                "RobotLocation: heading(%1$+.2f) pos(%2$.2f, %3$.2f)",
                getHeading_deg(),
                m_pos_in.x,
                m_pos_in.y);
    }

    public void updateHeadingFromImu(double imuHeading_deg) {
        m_fieldHeading_deg = imuHeading_deg + m_imuHeadingOffset_deg + m_headingCorrection_deg;
    }

    public void appendHeadingCorrection(double headingCorrection_deg) {
        m_headingCorrection_deg += headingCorrection_deg;
    }

    public void setPos(double x_in, double y_in) {
        m_pos_in.x = x_in;
        m_pos_in.y = y_in;
    }

    public Position getPos_in() {
        return m_pos_in;
    }

    public String getHeadingLogData() {
        return ", imuOffset, " + df3.format(m_imuHeadingOffset_deg) +
                "correction," + df3.format(m_headingCorrection_deg) +
                "fieldHeading, " + df3.format(m_fieldHeading_deg);
    }
}
