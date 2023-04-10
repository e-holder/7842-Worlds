package org.firstinspires.ftc.teamcode.middleware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HwVision;
import org.firstinspires.ftc.teamcode.middleware.VisionPipelineSignal.Signal;
import org.openftc.easyopencv.OpenCvPipeline;

public class Vision implements CONSTANTS {

    private final double CAMERA_TILT_SIGNAL = 0.0;
    // For the pole vision camera tilt, the following settings need to allow us to see just the pole
    // in our detection box without seeing any of a 4-stack on cones on that pole.
    private final double CAMERA_TILT_HIGH_POLE = 0.285;    // About 12 degrees up
    private final double CAMERA_TILT_MID_POLE = 0.33;      // About 30 degrees up

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

    public void setAlliance(Alliance alliance) {
        // TBD: May need to notify pipelines. E.g., m_barcodePipeline.setAlliance(alliance);
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
                setCameraTiltForSignal();
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

    // Camera will be about 19 inches (18-20) from High poles
    // Camera will be about 4 inches above the floor and about 45 degrees up from horizontal
    // The pipeline processing box should sit right on top of a stack of 4 cones on the pole.
    public static final int NOMINAL_HIGH_POLE_CENTER_PIX =
            (VisionPipelineFindPole.BOX_WIDTH / 2) + 48;
    public static final int NOMINAL_HIGH_POLE_WIDTH_PIX = 32;
    private final int MIN_HIGH_POLE_WIDTH_PIX = 26;

    // Camera will be about 10 inches (9-11) from Mid poles
    // Camera will be about 4 inches above the floor and about 30 degrees up from horizontal
    // The pipeline processing box should sit right on top of a stack of 4 cones on the pole.
    public static final int NOMINAL_MID_POLE_CENTER_PIX =
            (VisionPipelineFindPole.BOX_WIDTH / 2) + 51;
    public static final int NOMINAL_MID_POLE_WIDTH_PIX = 54;
    private final int MIN_MID_POLE_WIDTH_PIX = 46;

    // Constants to control how much heading change to score cones based on pole detection.
    private final double HIGH_PIX_TO_DEG = 0.1;  // TODO: Calibrate
    private final double MID_PIX_TO_DEG = 0.076;

    // Constants to control how much high pole lean influences heading adjustments.
    private final double HIGH_LEAN_AB_SCALE = 3.0 / (double)NOMINAL_HIGH_POLE_WIDTH_PIX;
    private final double HIGH_LEAN_AC_SCALE = 2.0 / (double)NOMINAL_HIGH_POLE_WIDTH_PIX;
    private final double HIGH_LEAN_BC_SCALE = 3.0 / (double)NOMINAL_HIGH_POLE_WIDTH_PIX;

    // Constants to control distance to score cones in autonomous based on pole width detection.
    private final double DEFAULT_SCORE_HIGH_DIST_IN = 4.0;
    private final double DEFAULT_SCORE_MID_DIST_IN = 3.0;
    private final double MAX_SCORE_HIGH_ADJUST_IN = 4.0;
    private final double MAX_SCORE_MID_ADJUST_IN = 4.0;
    private final double HIGH_WIDTH_PIX_TO_DIST_IN = -0.1; //TODO:Calibrate
    private final double MID_WIDTH_PIX_TO_DIST_IN = -0.25;

    // Ignore pole detections +/- this delta from nominal position.
    private final int MAX_DELTA_HIGH_PIX = 70;
    private final int MAX_DELTA_MID_PIX = 60;

    // Ignore pole detection if it is very different from other detections.
    private final int OUTLIER_HIGH_PIX = NOMINAL_HIGH_POLE_WIDTH_PIX * 3;
    private final int OUTLIER_MID_PIX = NOMINAL_MID_POLE_WIDTH_PIX * 2;

    // Ignore pole detection if it is far wider than expected.
    private final int SUPER_WIDE_HIGH_PIX = NOMINAL_HIGH_POLE_WIDTH_PIX * 3;
    private final int SUPER_WIDE_MID_PIX = NOMINAL_MID_POLE_WIDTH_PIX * 2;

    private PoleType m_poleType = PoleType.UNINITIALIZED;
    private final VisionPipelineFindPole m_findPolePipeline;
    private boolean m_isFindPoleStreaming = false;
    private boolean m_isFindPoleEnabled = false;
    private int m_findPoleFrameCount = -1;
    private int m_rowAPoleWidth_pix;
    private int m_rowBPoleWidth_pix;
    private int m_rowCPoleWidth_pix;
    private int m_rowAWidthDelta_pix;
    private int m_rowBWidthDelta_pix;
    private int m_rowCWidthDelta_pix;
    private int m_rowACol_pix;
    private int m_rowBCol_pix;
    private int m_rowCCol_pix;
    private int m_rowADelta_pix;
    private int m_rowBDelta_pix;
    private int m_rowCDelta_pix;
    private int m_detections;
    private double m_deltaToPole_deg;
    private double m_distToScore_in;

    // These variables and methods  are to allow a specific constant to be tuned live during a
    // test OpMode.  They have no effect unless "m_calibrationFactor" is substituted in for the
    // constant being tuned.
    private double m_calibrationFactor = -40.3;
    private double m_calibrationSmallStep = 0.01;
    private double m_calibrationBigStep = 1.0;
    public void calSmallStepUp() { m_calibrationFactor += m_calibrationSmallStep; }
    public void calSmallStepDown() { m_calibrationFactor -= m_calibrationSmallStep; }
    public void calBigStepUp() { m_calibrationFactor += m_calibrationBigStep; }
    public void calBigStepDown() { m_calibrationFactor -= m_calibrationBigStep; }

    private void setMinPoleWidth() {
        m_findPolePipeline.setMinPoleWidth((m_poleType == PoleType.HIGH ?
                MIN_HIGH_POLE_WIDTH_PIX :
                MIN_MID_POLE_WIDTH_PIX));
    }

    private void getFindPolePipelineInputs() {
        m_rowAPoleWidth_pix = m_findPolePipeline.getRowAPoleWidth_pix();
        m_rowBPoleWidth_pix = m_findPolePipeline.getRowBPoleWidth_pix();
        m_rowCPoleWidth_pix = m_findPolePipeline.getRowCPoleWidth_pix();
        m_rowACol_pix = m_findPolePipeline.getPoleRowACol_pix();
        m_rowBCol_pix = m_findPolePipeline.getPoleRowBCol_pix();
        m_rowCCol_pix = m_findPolePipeline.getPoleRowCCol_pix();
    }

    private void computeAllDeltaLateralPix() {
        int nominalCenter_pix = (m_poleType == PoleType.HIGH ?
                NOMINAL_HIGH_POLE_CENTER_PIX :
                NOMINAL_MID_POLE_CENTER_PIX);
        m_rowADelta_pix = m_rowACol_pix - nominalCenter_pix;
        m_rowBDelta_pix = m_rowBCol_pix - nominalCenter_pix;
        m_rowCDelta_pix = m_rowCCol_pix - nominalCenter_pix;
    }

    private void computeAllDeltaWidthPix() {
        int nominalWidth_pix = (m_poleType == PoleType.HIGH ?
                NOMINAL_HIGH_POLE_WIDTH_PIX :
                NOMINAL_MID_POLE_WIDTH_PIX);
        m_rowAWidthDelta_pix = m_rowAPoleWidth_pix - nominalWidth_pix;
        m_rowBWidthDelta_pix = m_rowBPoleWidth_pix - nominalWidth_pix;
        m_rowCWidthDelta_pix = m_rowCPoleWidth_pix - nominalWidth_pix;
    }

    private void eliminateFarLeftOrRightDetections() {
        int maxDelta_pix = (m_poleType == PoleType.HIGH ? MAX_DELTA_HIGH_PIX : MAX_DELTA_MID_PIX);
        if (Math.abs(m_rowADelta_pix) > maxDelta_pix) {
            m_detections &= 0x011;  // Remove A
        }
        if (Math.abs(m_rowBDelta_pix) > maxDelta_pix) {
            m_detections &= 0x101;  // Remove B
        }
        if (Math.abs(m_rowCDelta_pix) > maxDelta_pix) {
            m_detections &= 0x110;  // Remove C
        }
    }

    private void eliminateSuperWideDetections() {
        int superWide_pix = (m_poleType == PoleType.HIGH ?
                SUPER_WIDE_HIGH_PIX : SUPER_WIDE_MID_PIX);
        if (((m_detections & D_A) == D_A) && (m_rowAWidthDelta_pix > superWide_pix)) {
            m_detections &= 0x011;  // Remove A
        }
        if (((m_detections & D_B) == D_B) && (m_rowBWidthDelta_pix > superWide_pix)) {
            m_detections &= 0x101;  // Remove B
        }
        if (((m_detections & D_C) == D_C) && (m_rowCWidthDelta_pix > superWide_pix)) {
            m_detections &= 0x110;  // Remove C
        }
    }

    private void eliminateOutlierAmongThree(int outlier_pix) {
        int ab = Math.abs(m_rowADelta_pix - m_rowBDelta_pix);
        int ac = Math.abs(m_rowADelta_pix - m_rowCDelta_pix);
        int bc = Math.abs(m_rowBDelta_pix - m_rowCDelta_pix);
        int aOutlier = (ab + ac) / 2;
        int bOutlier = (ab + bc) / 2;
        int cOutlier = (ac + bc) / 2;
        if ((aOutlier > outlier_pix) && (aOutlier > bOutlier) && (aOutlier > cOutlier)) {
            // If A is an outlier, and worse than B and C, eliminate it.
            m_detections &= 0x011;  // Remove A
        } else if ((bOutlier > outlier_pix) && (bOutlier > aOutlier) && (bOutlier > cOutlier)) {
            // If B is an outlier, and worse than A and C, eliminate it.
            m_detections &= 0x101;  // Remove B
        } else if ((cOutlier > outlier_pix) && (cOutlier > aOutlier) && (cOutlier > bOutlier)) {
            // If C is an outlier, and worse than A and B, eliminate it.
            m_detections &= 0x110;  // Remove C
        }
    }

    private void eliminateOutlierAmongTwo(int outlier_pix) {
        switch (m_detections) {
            case D_AB:
                int ab = Math.abs(m_rowADelta_pix - m_rowBDelta_pix);
                if (ab > outlier_pix) {
                    if (Math.abs(m_rowADelta_pix) > Math.abs(m_rowBDelta_pix)) {
                        m_detections &= 0x011;  // Remove A
                    } else {
                        m_detections &= 0x101;  // Remove B
                    }
                }
                break;
            case D_AC:
                int ac = Math.abs(m_rowADelta_pix - m_rowCDelta_pix);
                if (ac > outlier_pix) {
                    if (Math.abs(m_rowADelta_pix) > Math.abs(m_rowCDelta_pix)) {
                        m_detections &= 0x011;  // Remove A
                    } else {
                        m_detections &= 0x110;  // Remove C
                    }
                }
                break;
            case D_BC:
                int bc = Math.abs(m_rowBDelta_pix - m_rowCDelta_pix);
                if (bc > outlier_pix) {
                    if (Math.abs(m_rowBDelta_pix) > Math.abs(m_rowCDelta_pix)) {
                        m_detections &= 0x101;  // Remove B
                    } else {
                        m_detections &= 0x110;  // Remove C
                    }
                }
                break;
        }
    }

    private double computeOneDetectionDeltaAngle_deg(int deltaPix) {
        // TODO: Test and calibrate for HIGH poles.
        return deltaPix * (m_poleType == PoleType.HIGH ? HIGH_PIX_TO_DEG : MID_PIX_TO_DEG);
    }

    private double computeOneDetectionDistToScore_in(int detection, int deltaWidthPix) {
        double defaultDist_in = (m_poleType == PoleType.HIGH ?
                DEFAULT_SCORE_HIGH_DIST_IN : DEFAULT_SCORE_MID_DIST_IN);
        double distAdjust_in;
        if (m_poleType == PoleType.HIGH) {
            // TODO: Test and calibrate for HIGH poles
            distAdjust_in = Math.max(-2.0, deltaWidthPix * HIGH_WIDTH_PIX_TO_DIST_IN);
            distAdjust_in = Math.min(distAdjust_in, MAX_SCORE_HIGH_ADJUST_IN);
        } else {
            switch (detection) {  // If using a higher detection, account for width delta.
                case D_B: deltaWidthPix += 3; break;
                case D_C: deltaWidthPix += 7; break;
            }
            distAdjust_in = Math.max(-2.0, deltaWidthPix * MID_WIDTH_PIX_TO_DIST_IN);
            distAdjust_in = Math.min(distAdjust_in, MAX_SCORE_MID_ADJUST_IN);
        }
        return defaultDist_in + distAdjust_in;
    }

    private double computeTwoDetectionDeltaToPole_deg(int detectionsUsed) {
        // TODO: Test & Calibrate
        double baseDelta_pix = 0.0;
        double leanDelta_pix = 0.0;
        switch (detectionsUsed) {
            case D_AB:
                baseDelta_pix = m_rowBDelta_pix;
                leanDelta_pix = (m_rowBDelta_pix - m_rowADelta_pix) * HIGH_LEAN_AB_SCALE;
                break;
            case D_AC:
                baseDelta_pix = m_rowCDelta_pix;
                leanDelta_pix = (m_rowCDelta_pix - m_rowADelta_pix) * HIGH_LEAN_AC_SCALE;
                break;
            case D_BC:
                baseDelta_pix = m_rowCDelta_pix;
                leanDelta_pix = (m_rowCDelta_pix - m_rowBDelta_pix) * HIGH_LEAN_BC_SCALE;
                break;
        }
        // Do not use leaning to adjust MID poles.
        leanDelta_pix = (m_poleType == PoleType.HIGH ? leanDelta_pix : 0.0);
        int delta_pix = (int)(baseDelta_pix + leanDelta_pix);
        return computeOneDetectionDeltaAngle_deg(delta_pix);
    }

    private double computeTwoDetectionDistToScore_in(int detectionsUsed) {
        int detection;
        int deltaWidth_pix;
        switch (detectionsUsed) {
            case D_BC:
                detection = D_B;
                deltaWidth_pix = m_rowBWidthDelta_pix;
                break;
            case D_AB:
            case D_AC:
            default:
                detection = D_A;
                deltaWidth_pix = m_rowAWidthDelta_pix;
                break;
        }
        return computeOneDetectionDistToScore_in(detection, deltaWidth_pix);
    }

    private double computeThreeDetectionDeltaToPole_deg() {
        // TODO: Test & Calibrate. Could use B detection to refine detection.
        double baseDelta_pix = m_rowCDelta_pix;
        double leanDelta_pix = (m_rowCDelta_pix - m_rowADelta_pix) * HIGH_LEAN_AC_SCALE;
        // Do not use leaning to adjust MID poles.
        leanDelta_pix = (m_poleType == PoleType.HIGH ? leanDelta_pix : 0.0);
        int delta_pix = (int)(baseDelta_pix + leanDelta_pix);
        return computeOneDetectionDeltaAngle_deg(delta_pix);
    }

    private double computeThreeDetectionDistToScore_in() {
        return computeOneDetectionDistToScore_in(D_A, m_rowAWidthDelta_pix);
    }

    private void computeFindPoleNavigation() {
        computeAllDeltaLateralPix();
        computeAllDeltaWidthPix();

        // TODO: Starting with just test/calibrate of MID pole "A detection" only.
//        m_detections = D_A;
        m_detections = D_ABC; // Initially assume all detections will be used.
        eliminateFarLeftOrRightDetections();
        eliminateSuperWideDetections();

        int outlier_pix = (m_poleType == PoleType.HIGH ? OUTLIER_HIGH_PIX : OUTLIER_MID_PIX);
        if (m_detections == D_ABC) {
            eliminateOutlierAmongThree(outlier_pix);
        }
        if (m_detections == D_AB || m_detections == D_AC || m_detections == D_BC) {
            eliminateOutlierAmongTwo(outlier_pix);
        }

        switch (m_detections) {
            case D_ABC:
                m_deltaToPole_deg = computeThreeDetectionDeltaToPole_deg();
                m_distToScore_in = computeThreeDetectionDistToScore_in();
                break;
            case D_AB:
                m_deltaToPole_deg = computeTwoDetectionDeltaToPole_deg(D_AB);
                m_distToScore_in = computeTwoDetectionDistToScore_in(D_AB);
                break;
            case D_AC:
                m_deltaToPole_deg = computeTwoDetectionDeltaToPole_deg(D_AC);
                m_distToScore_in = computeTwoDetectionDistToScore_in(D_AC);
                break;
            case D_BC:
                m_deltaToPole_deg = computeTwoDetectionDeltaToPole_deg(D_BC);
                m_distToScore_in = computeTwoDetectionDistToScore_in(D_BC);
                break;
            case D_A:
                m_deltaToPole_deg = computeOneDetectionDeltaAngle_deg(m_rowADelta_pix);
                m_distToScore_in = computeOneDetectionDistToScore_in(D_A, m_rowAWidthDelta_pix);
                break;
            case D_B:
                m_deltaToPole_deg = computeOneDetectionDeltaAngle_deg(m_rowBDelta_pix);
                m_distToScore_in = computeOneDetectionDistToScore_in(D_B, m_rowBWidthDelta_pix);
                break;
            case D_C:
                m_deltaToPole_deg = computeOneDetectionDeltaAngle_deg(m_rowCDelta_pix);
                m_distToScore_in = computeOneDetectionDistToScore_in(D_C, m_rowCWidthDelta_pix);
                break;
            default:
                m_deltaToPole_deg = 0.0;
                m_distToScore_in = (m_poleType == PoleType.HIGH ?
                        DEFAULT_SCORE_HIGH_DIST_IN : DEFAULT_SCORE_MID_DIST_IN);
                break;
        }
    }

    public int getFindPoleFrameCount() { return m_findPoleFrameCount; }

    public boolean isFindPolePipelineFrameBlack() {
        return m_findPolePipeline.isFrameBlack();
    }

    public int getDetections() { return m_detections; }

    public double getDeltaToPole_deg() {
        return m_deltaToPole_deg;
    }

    public double getDistToScore_in() {
        return m_distToScore_in;
    }

    public void setPoleType(PoleType newPoleType) {
        // TODO: If we have time, we should also set tilt angle based on how many cones are on the
        //  pole we are looking at. Note: This would affect distance to score calibrations.
        if (m_poleType != newPoleType) {
            m_poleType = newPoleType;
            setMinPoleWidth();
            if (newPoleType == PoleType.HIGH) {
                setCameraTiltForHighPole();
            } else {
                setCameraTiltForMidPole();
            }
            logCsvString("Vision, setPoleType, " + m_poleType);
        }
    }

    public void findPoleEnable(boolean isEnabled) {
        m_isFindPoleEnabled = isEnabled;
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
    // Camera Tilt Servo Functionality

    public void setCameraTiltForSignal() {
        m_hwVision.setCameraTilt(CAMERA_TILT_SIGNAL);
    }

    public void setCameraTiltForMidPole() {
        m_hwVision.setCameraTilt(CAMERA_TILT_MID_POLE);
    }

    public void setCameraTiltForHighPole() {
        m_hwVision.setCameraTilt(CAMERA_TILT_HIGH_POLE);
    }

    //============================================================================================

    public void reportData(Telemetry telemetry) {
        if (m_isSignalStreaming) {
            telemetry.addData("Park = ", m_parkingZone +
                    ", Signal = " + m_signal);
            telemetry.addData("averages", (int)m_signalBoxAvg +
                            " (" + (int)m_signalAvgTop + " / " + (int)m_signalAvgBottom + ")");
        }

        if (Vera.isVisionTestMode && m_isFindPoleStreaming) {
            telemetry.addData("Pole",
                    "DEG " + df3.format(m_deltaToPole_deg) +
                            ", IN " + df3.format(m_distToScore_in));
            if ((m_detections & D_A) == D_A) {
                telemetry.addData("  A deltas:", m_rowADelta_pix +
                        " width " + m_rowAWidthDelta_pix);
            }
            if ((m_detections & D_B) == D_B) {
                telemetry.addData("  B deltas:", m_rowBDelta_pix +
                        " width " + m_rowBWidthDelta_pix);
            }
            if ((m_detections & D_C) == D_C) {
                telemetry.addData("  C deltas:", m_rowCDelta_pix +
                        " width " + m_rowCWidthDelta_pix);
            }
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
        logCsvString("FindPole" +
                ", frame, " + m_findPoleFrameCount +
                ", LPFrame, " + df3.format(loopsPerFrame) +
                ", status, " + status +
                ", det, " + m_detections +
                ", deltaDeg, " + df3.format(m_deltaToPole_deg) +
                ", toScoreIn, " + df3.format(m_distToScore_in) +
                ", ACol, " + m_rowACol_pix +
//                    ", BCol, " + m_rowBCol_pix +
//                    ", CCol, " + m_rowCCol_pix +
                ", AWidth, " + m_rowAPoleWidth_pix +
//                    ", BWidth, " + m_rowBPoleWidth_pix +
//                    ", CWidth, " + m_rowCPoleWidth_pix +
                ", ADelta, " + m_rowADelta_pix +
//                    ", BDelta, " + m_rowBDelta_pix +
//                    ", CDelta, " + m_rowCDelta_pix +
                ", ADeltaW, " + m_rowAWidthDelta_pix +
//                    ", BDeltaW, " + m_rowBWidthDelta_pix +
//                    ", CDeltaW, " + m_rowCWidthDelta_pix +
                ", calF, " + df3.format(m_calibrationFactor) +
                "");
    }
}
