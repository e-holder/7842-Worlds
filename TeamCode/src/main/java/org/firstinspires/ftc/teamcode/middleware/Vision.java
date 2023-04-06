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

    // SUBSYSTEM has an instance of its corresponding hardware class here.
    private final HwVision m_hwVision;

    private final StringBuilder m_csvLogStr = new StringBuilder();

    // SUBSYSTEM has a public constructor here.
    public Vision(Vera vera, Telemetry telemetry) {

        // SUBSYSTEM constructs its corresponding hardware class instance here.
        m_hwVision = vera.getHwVision();

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

    public void logCsvString(String record) { m_csvLogStr.append(record).append("\n"); }

    public StringBuilder getLogString() {
        m_csvLogStr.append(m_findPolePipeline.getLogString());
        return m_csvLogStr;
    }

    private boolean m_isFirstPipeline = true;
    public void startStreaming(VeraPipelineType veraPipelineType) {
        OpenCvPipeline pipeline;
        switch (veraPipelineType) {
            case FIND_POLE:
                pipeline = m_findPolePipeline;
                setMinPoleWidth();
                m_findPoleUpdateCount = -1;
                m_findPoleFrameCount = -1;
                m_isFindPoleStreaming = true;
                m_isSignalStreaming = false;
                break;
            case SIGNAL:   // Intentional fall-through
            default:
                setCameraTiltForSignal();
                pipeline = m_signalPipeline;
                m_signalUpdateCount = -1;
                m_signalFrameCount = -1;
                m_isSignalStreaming = true;
                m_isFindPoleStreaming = false;
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
        double signalLoopPerFrame = (double)m_signalUpdateCount / (double)m_signalFrameCount;
        logCsvString("SignalVision" +
                ", loopPerFrame, " + df3.format(signalLoopPerFrame));
        double findPoleLoopPerFrame = (double)m_findPoleUpdateCount / (double)m_findPoleFrameCount;
        logCsvString("FindPoleVision" +
                ", loopPerFrame, " + df3.format(findPoleLoopPerFrame));
    }

    public void getInputs() {
        int newFrameCount;
        if (m_isSignalStreaming) {
            m_signalUpdateCount++;
            newFrameCount = m_signalPipeline.getFrameCount();
            if (m_signalFrameCount != newFrameCount) {
                m_signalFrameCount = newFrameCount;
                getSignalPipelineInputs();
            }
        } else if (m_isFindPoleStreaming) {
            m_findPoleUpdateCount++;
            newFrameCount = m_findPolePipeline.getFrameCount();
            if (m_findPoleFrameCount != newFrameCount) {
                m_findPoleFrameCount = newFrameCount;
                getFindPolePipelineInputs();
                computeFindPoleNavigation();
            }
        }
    }

    //============================================================================================
    // Find Pole Vision Functionality

    // Camera will be about 19 inches (18-20) from High poles
    // Camera will be about 4 inches above the floor and about 45 degrees up from horizontal
    // The pipeline processing box should sit right on top of a stack of 4 cones on the pole.
    public static final int NOMINAL_HIGH_POLE_CENTER_PIX = VisionPipelineFindPole.BOX_WIDTH/2 + 48;
    public static final int NOMINAL_HIGH_POLE_WIDTH_PIX = 32;
    private final int MIN_HIGH_POLE_WIDTH_PIX = 26;

    // Camera will be about 10 inches (9-11) from Mid poles
    // Camera will be about 4 inches above the floor and about 30 degrees up from horizontal
    // The pipeline processing box should sit right on top of a stack of 4 cones on the pole.
    public static final int NOMINAL_MID_POLE_CENTER_PIX = VisionPipelineFindPole.BOX_WIDTH/2 + 51;
    public static final int NOMINAL_MID_POLE_WIDTH_PIX = 54;
    private final int MIN_MID_POLE_WIDTH_PIX = 46;

    private PoleType m_poleType = PoleType.MID;
    private final VisionPipelineFindPole m_findPolePipeline;
    private boolean m_isFindPoleStreaming = false;
    private int m_findPoleUpdateCount = 0;
    private int m_findPoleFrameCount = -1;
    private boolean m_areFindPoleFramesValid = false;
    private int m_rowAPoleWidth;
    private int m_rowBPoleWidth;
    private int m_rowCPoleWidth;
    private int m_rowACol_pix;
    private int m_rowBCol_pix;
    private int m_rowCCol_pix;
    private int m_rowDetections;
    private double m_deltaToPole_deg;
    private double m_distToScore_in;

    private void getFindPolePipelineInputs() {
        m_areFindPoleFramesValid = !m_findPolePipeline.isFrameNull();
        m_rowAPoleWidth = m_findPolePipeline.getRowAPoleWidth_pix();
        m_rowBPoleWidth = m_findPolePipeline.getRowBPoleWidth_pix();
        m_rowCPoleWidth = m_findPolePipeline.getRowCPoleWidth_pix();
        m_rowACol_pix = m_findPolePipeline.getPoleRowACol_pix();
        m_rowBCol_pix = m_findPolePipeline.getPoleRowBCol_pix();
        m_rowCCol_pix = m_findPolePipeline.getPoleRowCCol_pix();
    }

    public int getFindPoleFrameCount() { return m_findPoleFrameCount; }
    public boolean areFindPoleFramesValid() {
        return m_areFindPoleFramesValid;
    }
    public int getRowAPoleWidth_pix() { return m_rowAPoleWidth; }
    public int getRowBPoleWidth_pix() { return m_rowBPoleWidth; }
    public int getRowCPoleWidth_pix() { return m_rowCPoleWidth; }
    public double getDeltaToPole_deg() { return m_deltaToPole_deg; }
    public double getDistToScore_in() {
        return m_distToScore_in;
    }

    public int getPoleAOffset_pix() {
        return m_rowACol_pix - (m_poleType == PoleType.HIGH ?
                NOMINAL_HIGH_POLE_CENTER_PIX :
                NOMINAL_MID_POLE_CENTER_PIX);
    }

    public int getPoleBOffset_pix() {
        return m_rowBCol_pix - (m_poleType == PoleType.HIGH ?
                NOMINAL_HIGH_POLE_CENTER_PIX :
                NOMINAL_MID_POLE_CENTER_PIX);
    }

    public int getPoleCOffset_pix() {
        return m_rowCCol_pix - (m_poleType == PoleType.HIGH ?
                NOMINAL_HIGH_POLE_CENTER_PIX :
                NOMINAL_MID_POLE_CENTER_PIX);
    }

    public int getNominalPoleWidth() {
        return (m_poleType == PoleType.HIGH ?
                NOMINAL_HIGH_POLE_WIDTH_PIX :
                NOMINAL_MID_POLE_WIDTH_PIX);
    }

    private void computeFindPoleNavigation() {
        m_rowDetections = 0;
        if (m_rowACol_pix >= 0) m_rowDetections++;
        if (m_rowBCol_pix >= 0) m_rowDetections++;
        if (m_rowCCol_pix >= 0) m_rowDetections++;

        // Compute delta degrees toward pole.
        m_deltaToPole_deg = 0.0;

        // Compute distance to score pole.
        m_distToScore_in = 2.0;
    }

    public void setPoleType(PoleType newPoleType) {
        m_poleType = newPoleType;
        setMinPoleWidth();
        if (newPoleType == PoleType.HIGH) {
            setCameraTiltForHighPole();
        } else {
            setCameraTiltForMidPole();
        }
        logCsvString("Vision, setPoleType, " + m_poleType);
    }

    public void setMinPoleWidth() {
        m_findPolePipeline.setMinPoleWidth((m_poleType == PoleType.HIGH ?
                MIN_HIGH_POLE_WIDTH_PIX :
                MIN_MID_POLE_WIDTH_PIX));
    }

    //============================================================================================
    // Signal Vision Functionality

    private final VisionPipelineSignal m_signalPipeline;
    private boolean m_isSignalStreaming = false;
    private int m_signalUpdateCount = 0;
    private int m_signalFrameCount = -1;
    private double m_signalBoxAvg;
    private double m_signalAvgTop;
    private double m_signalAvgBottom;
    private Signal m_signal = Signal.UNKNOWN;
    private Signal m_parkingZone = Signal.ZONE3;

    private void getSignalPipelineInputs() {
        m_signalBoxAvg = m_signalPipeline.getSignalBoxAvg();
        m_signalAvgTop = m_signalPipeline.getSignalTopAvg();
        m_signalAvgBottom = m_signalPipeline.getSignalBottomAvg();
        m_signal = m_signalPipeline.getSignal();
        m_parkingZone = (m_signal == Signal.UNKNOWN ? Signal.ZONE3 : m_signal);
    }

    public boolean areSignalImagesValid() {
        return m_signalAvgTop > 0.0;
    }

    // Accessors for signal pipeline results.
    public int getSignalPipelineFrameCount() { return m_signalFrameCount; }
    public Signal getSignal() { return m_signal; }

    public Signal getParkingZone() {
        logCsvString("Vision" +
                ", Signal, " + m_signal +
                ", Park, " + m_parkingZone);
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
        if (true && m_isSignalStreaming) {
            telemetry.addData("Park = ", m_parkingZone +
                    ", Signal = " + m_signal);
            telemetry.addData("averages", (int)m_signalBoxAvg +
                            " (" + (int)m_signalAvgTop + " / " + (int)m_signalAvgBottom + ")");
        }

        if (false) {
            logCsvString("Signal, " + m_signal +
                    ", avgBox, " + df3.format(m_signalBoxAvg) +
                    ", avgTop, " + df3.format(m_signalAvgTop) +
                    ", avgBottom, " + df3.format(m_signalAvgBottom) +
                    ", frame, " + m_signalFrameCount);
        }

        if (true && Vera.isVisionTestMode && m_isFindPoleStreaming) {
            // TODO: Fix
//            telemetry.addData("Pole A ",
//                    "upper " + getPoleUpperCol() +
//                    " lower " + getPoleLowerCol() +
//                    " width " + getUpperPoleWidth());
        }
    }
}
