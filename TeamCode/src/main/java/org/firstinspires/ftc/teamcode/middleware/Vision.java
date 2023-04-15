package org.firstinspires.ftc.teamcode.middleware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HwVision;
import org.firstinspires.ftc.teamcode.middleware.VisionPipelineSignal.Signal;
import org.openftc.easyopencv.OpenCvPipeline;

public class Vision implements CONSTANTS {

    private final double CAMERA_TILT_SIGNAL = 0.0;
    // For the pole vision camera tilt, the following settings need to allow us to see just the pole
    // in our detection box without seeing any of a 4-stack of cones on that pole.
    // 0.285 = about 12 deg
    // 0.33 = about 30 deg
    private final double CAMERA_TILT_HIGH_POLE = 0.33;
    private final double CAMERA_TILT_MID_POLE = 0.33;
    private final double CAMERA_TILT_HIGH_SCORED_CONES = 0.0;
    private final double CAMERA_TILT_MID_SCORED_CONES = 0.0;

    private final Vera m_vera;
    private final StringBuilder m_csvLogStr = new StringBuilder();

    // SUBSYSTEM has an instance of its corresponding hardware class here.
    private final HwVision m_hwVision;

    // SUBSYSTEM has a public constructor here.
    public Vision(Vera vera, Telemetry telemetry) {
        m_vera = vera;

        // SUBSYSTEM constructs its corresponding hardware class instance here.
        m_hwVision = m_vera.getHwVision();

        // Update startStreaming if list of pipelines change
        m_signalPipeline = new VisionPipelineSignal(telemetry);
        m_findPolePipeline = new VisionPipelineFindPole(telemetry);
    }

    // SUBSYSTEM has a private constructor here.
    private Vision() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Default constructor not supported.");
    }

    public void logCsvString(String record) {
        m_csvLogStr.append(record).append("\n");
    }

    public StringBuilder getLogString() {
        m_csvLogStr.append(m_findPolePipeline.getLogString());
        return m_csvLogStr;
    }

    public boolean isSignalStreaming() { return m_isSignalStreaming; }
    public boolean isFindPoleStreaming() { return m_isFindPoleStreaming; }

    private boolean m_isFirstPipeline = true;
    public void startStreaming(VeraPipelineType veraPipelineType) {
        OpenCvPipeline pipeline;
        switch (veraPipelineType) {
            case SIGNAL:
                m_hwVision.setCameraTilt(CAMERA_TILT_SIGNAL);
                pipeline = m_signalPipeline;
                m_startSignalLoopCount = m_vera.getLoopCount();
                m_isSignalStreaming = true;
                m_isFindPoleStreaming = false;
                break;
            case FIND_POLE:    // Intentional fall-through
            default:
                // If FIND_POLE is starting, then SIGNAL is stopping. Log its loops per frame.
                double signalLoopPerFrame =
                        (double)(m_vera.getLoopCount() - m_startSignalLoopCount) /
                        (double)m_signalFrameCount;
                logCsvString("SignalVision: loopPerFrame, " +
                        df3.format(signalLoopPerFrame));

                // Now get the FIND_POLE pipeline going
                pipeline = m_findPolePipeline;
                m_isFindPoleStreaming = true;
                m_isSignalStreaming = false;
                break;
        }

        if (m_isFirstPipeline) {
            // 'startWebcamStreaming' is used the first time
            m_hwVision.startWebcamStreaming(pipeline);
            m_isFirstPipeline = false;
        } else {
            // 'changeWebcamPipeline' is used for subsequent pipelines
            m_hwVision.changeWebcamPipeline(pipeline);
        }
        logCsvString("Pipeline = " + pipeline.toString());
    }

    public void stopWebcamStreaming() {
        m_hwVision.stopWebcamStreaming();
        m_isFindPoleStreaming = false;
        m_isSignalStreaming = false;
    }

    public void getInputs() {
        int newFrameCount;
        if (m_isSignalStreaming) {
            newFrameCount = m_signalPipeline.getFrameCount();
            if (m_signalFrameCount != newFrameCount) {
                m_signalFrameCount = newFrameCount;
                getSignalPipelineInputs();
            }
        } else if (m_isFindPoleStreaming) {
            newFrameCount = m_findPolePipeline.getFrameCount();
            if (m_findPoleFrameCount != newFrameCount) {
                m_findPoleFrameCount = newFrameCount;
                if (m_isFindPoleEnabled) {
                    getFindPolePipelineInputs();
                    computeFindPoleNavigation();
                }
            }
        }
    }

    //============================================================================================
    // Find Pole Vision Functionality

    // Camera would ideally be about 19 inches from High poles
    // Camera will be about 4 inches above the floor and about 45 degrees up from horizontal
    // The pipeline processing box should sit right on top of a stack of 4 cones on the pole.
    public static final int NOMINAL_HIGH_POLE_CENTER_PIX =
            (VisionPipelineFindPole.BOX_WIDTH / 2) + 62;
    private final int MAX_HIGH_POLE_WIDTH_PIX = 120;
    public static final int NOMINAL_HIGH_POLE_WIDTH_PIX = 32;
    private final int MIN_HIGH_POLE_WIDTH_PIX = 10;

    // TODO: Calibrate these three
    public static final int NOMINAL_HIGH_CONES_CENTER_PIX =
            (VisionPipelineFindPole.CONE_BOX_WIDTH / 2) + 62;
    private final int MAX_HIGH_CONES_WIDTH_PIX = VisionPipelineFindPole.CONE_BOX_WIDTH;
    public static final int NOMINAL_HIGH_CONES_WIDTH_PIX = 80;
    private final int MIN_HIGH_CONES_WIDTH_PIX = 50;

    // Camera would ideally be about 10 inches from Mid poles
    // Camera will be about 4 inches above the floor and about 30 degrees up from horizontal
    // The pipeline processing box should sit right on top of a stack of 4 cones on the pole.
    public static final int NOMINAL_MID_POLE_CENTER_PIX =
            (VisionPipelineFindPole.BOX_WIDTH / 2) - 5;
    private final int MAX_MID_POLE_WIDTH_PIX = 120;
    public static final int NOMINAL_MID_POLE_WIDTH_PIX = 86;
    private final int MIN_MID_POLE_WIDTH_PIX = 47;  // TODO: Note this is too high if D > 10 in

    public static final int NOMINAL_MID_CONES_CENTER_PIX =
            (VisionPipelineFindPole.CONE_BOX_WIDTH / 2) - 5;
    private final int MAX_MID_CONES_WIDTH_PIX = VisionPipelineFindPole.CONE_BOX_WIDTH;
    public static final int NOMINAL_MID_CONES_WIDTH_PIX = 360;
    private final int MIN_MID_CONES_WIDTH_PIX = 80;

    // Constants to control how much heading change to score cones based on pole detection.
    private final double HIGH_PIX_TO_DEG = 0.088;
    private final double MID_PIX_TO_DEG = 0.076;

//     Constants to control how much high pole lean influences heading adjustments.
//    private final double HIGH_LEAN_AC_SCALE = 3.5;
//    private final double HIGH_LEAN_AB_SCALE = HIGH_LEAN_AC_SCALE * 0.5;
//    private final double HIGH_LEAN_BC_SCALE = HIGH_LEAN_AC_SCALE * 0.5;

    // Constants to control distance to score cones in autonomous based on pole width detection.
//    private final double DEFAULT_SCORE_HIGH_DIST_IN = 4.0;
//    private final double DEFAULT_SCORE_MID_DIST_IN = 3.0;
//    private final double MAX_SCORE_HIGH_ADJUST_IN = 4.0;
//    private final double MAX_SCORE_MID_ADJUST_IN = 4.0;
    private final double HIGH_WIDTH_PIX_TO_DIST_IN = -1.001;
    private final double HIGH_CONES_WIDTH_PIX_TO_DIST_IN = 0.0000001; // TODO: Calibrate
    private final double MID_WIDTH_PIX_TO_DIST_IN = -(3.0 / 18.0);        // Inches / Pixel Change
    private final double MID_CONES_WIDTH_PIX_TO_DIST_IN = -(3.0 / 100.0); // Inches / Pixel Change

    // Ignore pole detections +/- this delta from nominal position.
    private final int MAX_DELTA_HIGH_PIX = 70;
    private final int MAX_DELTA_MID_PIX = 60;

    private FindPoleMode m_findPoleMode = FindPoleMode.UNINITIALIZED;
    private final VisionPipelineFindPole m_findPolePipeline;
    private boolean m_isFindPoleStreaming = false;
    private boolean m_isFindPoleEnabled = false;
    private boolean m_isPoleDetected = false;
    private int m_findPoleFrameCount = -1;
    private int m_poleWidth_pix;
    private int m_widthDelta_pix;
    private int m_poleCol_pix;
    private int m_poleColDelta_pix;
    private double m_deltaToPole_deg;
    private double m_distToScore_in;

    // These variables and methods  are to allow a specific constant to be tuned live during a
    // test OpMode.  They have no effect unless "m_calibrationFactor" is substituted in for the
    // constant being tuned.
    private double m_calibrationFactor = 3.5;
    private final double m_calibrationSmallStep = 0.01;
    private final double m_calibrationBigStep = 0.1;
    public void calSmallStepUp() { m_calibrationFactor += m_calibrationSmallStep; }
    public void calSmallStepDown() { m_calibrationFactor -= m_calibrationSmallStep; }
    public void calBigStepUp() { m_calibrationFactor += m_calibrationBigStep; }
    public void calBigStepDown() { m_calibrationFactor -= m_calibrationBigStep; }

    private void getFindPolePipelineInputs() {
        m_poleWidth_pix = m_findPolePipeline.getPoleWidth_pix();
        m_poleCol_pix = m_findPolePipeline.getPoleCol_pix();
    }

    private boolean computeDeltaLateralPix() {
        int nominalCenter_pix = (m_findPoleMode == FindPoleMode.HIGH_POLE ?
                NOMINAL_HIGH_POLE_CENTER_PIX :
                NOMINAL_MID_POLE_CENTER_PIX);
        m_poleColDelta_pix = m_poleCol_pix - nominalCenter_pix;
        return (m_poleCol_pix >= 0);
    }

    private boolean computeDeltaWidthPix() {
        switch (m_findPoleMode) {
            case HIGH_POLE:
                m_widthDelta_pix = m_poleWidth_pix - NOMINAL_HIGH_POLE_WIDTH_PIX;
                break;
            case MID_POLE:
                m_widthDelta_pix = m_poleWidth_pix - NOMINAL_MID_POLE_WIDTH_PIX;
                break;
            case HIGH_SCORED_CONES:
                m_widthDelta_pix = m_poleWidth_pix - NOMINAL_HIGH_CONES_WIDTH_PIX;
                break;
            case MID_SCORED_CONES:
                m_widthDelta_pix = m_poleWidth_pix - NOMINAL_MID_CONES_WIDTH_PIX;
                break;
        }
        return (m_poleWidth_pix >= 0);
    }

    private double computeDeltaAngle_deg(int deltaPix) {
        double deltaAngle_deg;
        switch (m_findPoleMode) {
            case HIGH_POLE:
            case HIGH_SCORED_CONES:
                deltaAngle_deg =  deltaPix * HIGH_PIX_TO_DEG;
                break;
            case MID_POLE:
            case MID_SCORED_CONES:
            default:
                deltaAngle_deg =  deltaPix * MID_PIX_TO_DEG;
                break;
        }
        return deltaAngle_deg;
    }

    private double computeDistToScore_in(int deltaWidthPix) {
        double distToScore_in;
        switch (m_findPoleMode) {
            case HIGH_POLE:
                distToScore_in = deltaWidthPix * HIGH_WIDTH_PIX_TO_DIST_IN;
                break;
            case HIGH_SCORED_CONES:
                distToScore_in =  deltaWidthPix * HIGH_CONES_WIDTH_PIX_TO_DIST_IN;
                break;
            case MID_POLE:
                distToScore_in = deltaWidthPix * MID_WIDTH_PIX_TO_DIST_IN;
                break;
            case MID_SCORED_CONES:
            default:
                distToScore_in =  deltaWidthPix * MID_CONES_WIDTH_PIX_TO_DIST_IN;
                break;
        }
        return distToScore_in;
    }

    private void computeFindPoleNavigation() {
        m_isPoleDetected = computeDeltaLateralPix() && computeDeltaWidthPix();
        if (m_isPoleDetected) {
            m_deltaToPole_deg = computeDeltaAngle_deg(m_poleColDelta_pix);
            m_distToScore_in = computeDistToScore_in(m_widthDelta_pix);
        } else {
            m_deltaToPole_deg = 0.0;
            m_distToScore_in = 0.0;
        }
    }

    public int getFindPoleFrameCount() { return m_findPoleFrameCount; }

    public boolean isFindPolePipelineFrameBlack() {
        return m_findPolePipeline.isFrameBlack();
    }

    public boolean isPoleDetected() { return m_isPoleDetected; }
    public double getDeltaToPole_deg() {
        return m_deltaToPole_deg;
    }
    public double getDistToScore_in() {
        return m_distToScore_in;
    }

    public void setFindPoleMode(FindPoleMode newFindPoleMode, String status) {
        logCsvString("Vision, setFindPoleMode, " + m_findPoleMode + " " + status);
        if (m_findPoleMode != newFindPoleMode) {
            m_findPoleMode = newFindPoleMode;
            switch (m_findPoleMode) {
                case MID_POLE:
                    m_hwVision.setCameraTilt(CAMERA_TILT_MID_POLE);
                    m_findPolePipeline.setFindPoleMode(FindPoleMode.MID_POLE,
                            m_vera.getAlliance());
                    m_findPolePipeline.setMinMaxWidth(
                            MIN_MID_POLE_WIDTH_PIX, MAX_MID_POLE_WIDTH_PIX);
                    break;
                case HIGH_POLE:
                    m_hwVision.setCameraTilt(CAMERA_TILT_HIGH_POLE);
                    m_findPolePipeline.setFindPoleMode(FindPoleMode.HIGH_POLE,
                            m_vera.getAlliance());
                    m_findPolePipeline.setMinMaxWidth(
                            MIN_HIGH_POLE_WIDTH_PIX, MAX_HIGH_POLE_WIDTH_PIX);
                    break;
                case MID_SCORED_CONES:
                    m_hwVision.setCameraTilt(CAMERA_TILT_MID_SCORED_CONES);
                    m_findPolePipeline.setFindPoleMode(FindPoleMode.MID_SCORED_CONES,
                            m_vera.getAlliance());
                    m_findPolePipeline.setMinMaxWidth(
                            MIN_MID_CONES_WIDTH_PIX, MAX_MID_CONES_WIDTH_PIX);
                    break;
                case HIGH_SCORED_CONES:
                default:
                    m_hwVision.setCameraTilt(CAMERA_TILT_HIGH_SCORED_CONES);
                    m_findPolePipeline.setFindPoleMode(FindPoleMode.HIGH_SCORED_CONES,
                            m_vera.getAlliance());
                    m_findPolePipeline.setMinMaxWidth(
                            MIN_HIGH_CONES_WIDTH_PIX, MAX_HIGH_CONES_WIDTH_PIX);
                    break;
            }
        }
    }

    public void findPoleEnable(boolean isEnabled) {
        m_isFindPoleEnabled = isEnabled;
//        logCsvString("Vision, isFindPoleEnabled, " + m_isFindPoleEnabled);
        if (!m_isFindPoleEnabled) {
            m_isPoleDetected = false;
        }
    }

    //============================================================================================
    // Signal Vision Functionality

    private final Signal DEFAULT_PARKING_ZONE = Signal.ZONE3;

    private final VisionPipelineSignal m_signalPipeline;

    private boolean m_isSignalStreaming = false;
    private int m_startSignalLoopCount;
    private int m_signalFrameCount = -1;
    private double m_signalBoxAvg;
    private double m_signalAvgTop;
    private double m_signalAvgBottom;
    private Signal m_signal = Signal.UNKNOWN;
    private Signal m_parkingZone = DEFAULT_PARKING_ZONE;

    private void getSignalPipelineInputs() {
        m_signalBoxAvg = m_signalPipeline.getSignalBoxAvg();
        m_signalAvgTop = m_signalPipeline.getSignalTopAvg();
        m_signalAvgBottom = m_signalPipeline.getSignalBottomAvg();
        m_signal = m_signalPipeline.getSignal();
        m_parkingZone = (m_signal == Signal.UNKNOWN ? DEFAULT_PARKING_ZONE : m_signal);
    }

    public boolean isSignalPipelineFrameBlack() {
        return m_signalAvgTop > 0.0;
    }

    // Accessors for signal pipeline results.
    public int getSignalPipelineFrameCount() { return m_signalFrameCount; }
    public Signal getSignal() { return m_signal; }

    public Signal getParkingZone() {
        logCsvString("Vision" +
                ", Signal, " + m_signal +
                ", Park, " + m_parkingZone +
                ", avgBox, " + df3.format(m_signalBoxAvg) +
                ", avgTop, " + df3.format(m_signalAvgTop) +
                ", avgBottom, " + df3.format(m_signalAvgBottom) +
                ", frame, " + m_signalFrameCount);

        return m_parkingZone;
    }

    //============================================================================================
    private void useSomeVariablesToSatisfyAndroidStudio() {
        double d = CAMERA_TILT_SIGNAL + CAMERA_TILT_HIGH_POLE + CAMERA_TILT_MID_POLE +
                MAX_HIGH_POLE_WIDTH_PIX + MIN_HIGH_POLE_WIDTH_PIX + MAX_MID_POLE_WIDTH_PIX +
                MIN_MID_POLE_WIDTH_PIX + HIGH_PIX_TO_DEG + MID_PIX_TO_DEG +
                HIGH_WIDTH_PIX_TO_DIST_IN + MID_WIDTH_PIX_TO_DIST_IN ;
    }

    //============================================================================================

    public void reportData(Telemetry telemetry) {
        if (m_isSignalStreaming) {
            telemetry.addData("Park = ", m_parkingZone +
                    ", Signal = " + m_signal);
            telemetry.addData("averages", (int)m_signalBoxAvg +
                            " (" + (int)m_signalAvgTop + " / " + (int)m_signalAvgBottom + ")");
        }

        if (m_isFindPoleEnabled) {
            telemetry.addData("Pole",
                    "DEG " + df3.format(m_deltaToPole_deg) +
                            ", IN " + df3.format(m_distToScore_in));
            telemetry.addData("  Deltas:",  m_poleColDelta_pix +
                        " width " + m_widthDelta_pix);
            telemetry.addData("calF", df3.format(m_calibrationFactor));
        }

        if (false) {
            logCsvString("Signal, " + m_signal +
                    ", avgBox, " + df3.format(m_signalBoxAvg) +
                    ", avgTop, " + df3.format(m_signalAvgTop) +
                    ", avgBottom, " + df3.format(m_signalAvgBottom) +
                    ", frame, " + m_signalFrameCount);
        }
    }

    public void logFindPoleData(String status, double loopsPerFrame) {
        logCsvString("FindPole: " + status +
                ", frame, " + m_findPoleFrameCount +
                ", detected, " + m_isPoleDetected +
                ", deltaDeg, " + df3.format(m_deltaToPole_deg) +
                ", toScoreIn, " + df3.format(m_distToScore_in) +
                ", Col, " + m_poleCol_pix +
                ", Width, " + m_poleWidth_pix +
                ", Delta, " + m_poleColDelta_pix +
                ", DeltaW, " + m_widthDelta_pix +
                ", LoopPerFrm, " + df3.format(loopsPerFrame) +
                ", calF, " + df3.format(m_calibrationFactor) +
                "");
    }
}
