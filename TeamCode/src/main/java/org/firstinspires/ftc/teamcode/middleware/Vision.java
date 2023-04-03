package org.firstinspires.ftc.teamcode.middleware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HwVision;
import org.openftc.easyopencv.OpenCvPipeline;

public class Vision implements CONSTANTS {

    private final double CAMERA_TILT_SIGNAL = 0.0;
    // For the pole vision camera tilt, the following settings need to allow us to see just the pole
    // in our detection box without seeing any of a 4-stack on cones on that pole.
    private final double CAMERA_TILT_HIGH_POLE = 0.285;
    private final double CAMERA_TILT_MED_POLE = 0.33;

    // SUBSYSTEM has an instance of its corresponding hardware class here.
    private HwVision m_hwVision;

    private StringBuilder m_csvLogStr = new StringBuilder();

    // SUBSYSTEM has a public constructor here.
    public Vision(Vera vera, Telemetry telemetry) {
        // SUBSYSTEM constructs its corresponding hardware class instance here.
        m_hwVision = vera.getHwVision();

        setCameraTiltForSignal();

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
                break;
            case SIGNAL:   // Intentional fall-through
            default:
                pipeline = m_signalPipeline;
                break;
        }
        if (m_isFirstPipeline) {
            // 'start' is used the first time
            m_hwVision.startWebcamStreaming(pipeline);
            // TODO: Nix
            logCsvString("Pipeline = " + pipeline.toString());
            m_isFirstPipeline = false;
        } else {
            // 'change' is used for subsequent pipelines
            m_hwVision.changeWebcamPipeline(pipeline);
        }
    }

    public void stopWebcamStreaming() {
        m_hwVision.stopWebcamStreaming();
    }

    //============================================================================================
    // Find Pole Vision Functionality

    // Note: Camera will be about 19.5 inches from High Poles, 10 inches from Medium Poles, and 2.5
    // inches from Low Poles.
    public static final int NOMINAL_HIGH_POLE_CENTER_PIX = VisionPipelineFindPole.BOX_WIDTH/2 + 0;
    public static final int NOMINAL_HIGH_POLE_WIDTH_PIX = 32;
    private final int MIN_HIGH_POLE_WIDTH_PIX = 24;

    public static final int NOMINAL_MED_POLE_CENTER_PIX = VisionPipelineFindPole.BOX_WIDTH/2 + 17;
    public static final int NOMINAL_MED_POLE_WIDTH_PIX = 60;
    private final int MIN_MED_POLE_WIDTH_PIX = 45;

    // TODO: Calibrate low pole constants (if we ever need them in autonomous)
    public static final int NOMINAL_LOW_POLE_CENTER_PIX = VisionPipelineFindPole.BOX_WIDTH/2 - 4;
    public static final int NOMINAL_LOW_POLE_WIDTH_PIX = 160;
    private final int MIN_LOW_POLE_WIDTH_PIX = 100;

    // This is public to allow test autonomous OpModes to set the pole type.
    public static CornerType poleType;

    private VisionPipelineFindPole m_findPolePipeline;

    public void setPoleType(CornerType newPoleType) {
        poleType = newPoleType;
        setMinPoleWidth();
    }

    public void setMinPoleWidth() {
        switch (poleType) {
            case H:
                m_findPolePipeline.setMinPoleWidth(MIN_HIGH_POLE_WIDTH_PIX);
                break;
            case M:
                m_findPolePipeline.setMinPoleWidth(MIN_MED_POLE_WIDTH_PIX);
                break;
            default:
                m_findPolePipeline.setMinPoleWidth(MIN_LOW_POLE_WIDTH_PIX);
                break;
        }
    }

    public boolean areFindPoleImagesValid() {
        return !m_findPolePipeline.isFrameNull();
    }

    public int getFindPolePipelineFrameCount() { return m_findPolePipeline.getFrameCount(); }

    public int getPoleUpperOffset_pix() {
        int offset_pix = m_findPolePipeline.getPoleUpperCol_pix();
        switch (poleType) {
            case H:
                offset_pix -= NOMINAL_HIGH_POLE_CENTER_PIX;
                break;
            case M:
                offset_pix -= NOMINAL_MED_POLE_CENTER_PIX;
                break;
            default:
                offset_pix -= NOMINAL_LOW_POLE_CENTER_PIX;
                break;
        }
        logCsvString("UpperOffset, " + offset_pix);
        return offset_pix;
    }

    public int getPoleLowerOffset_pix() {
        int offset_pix = m_findPolePipeline.getPoleLowerCol_pix();
        switch (poleType) {
            case H:
                offset_pix -= NOMINAL_HIGH_POLE_CENTER_PIX;
                break;
            case M:
                offset_pix -= NOMINAL_MED_POLE_CENTER_PIX;
                break;
            default:
                offset_pix -= NOMINAL_LOW_POLE_CENTER_PIX;
                break;
        }
        logCsvString("LowerOffset, " + offset_pix);
        return offset_pix;
    }

    public int getNominalPoleWidth() {
        int nominalWidth_pix;
        switch (poleType) {
            case H:
                nominalWidth_pix = NOMINAL_HIGH_POLE_WIDTH_PIX;
                break;
            case M:
                nominalWidth_pix = NOMINAL_MED_POLE_WIDTH_PIX;
                break;
            default:
                nominalWidth_pix = NOMINAL_LOW_POLE_WIDTH_PIX;
                break;
        }
        return nominalWidth_pix;
    }

    public int getUpperPoleWidth_pix() { return m_findPolePipeline.getUpperPoleWidth_pix(); }

    public int getLowerPoleWidth_pix() { return m_findPolePipeline.getLowerPoleWidth_pix(); }

    //============================================================================================
    // Signal Vision Functionality

    private VisionPipelineSignal m_signalPipeline;
    private VisionPipelineSignal.Signal m_signal = VisionPipelineSignal.Signal.UNKNOWN;

    public boolean areSignalImagesValid() {
        return m_signalPipeline.getSignalTopAvg() > 0.0;
    }

    // Accessors for signal results.
    public VisionPipelineSignal.Signal getSignal() { return m_signalPipeline.getSignal(); }
    public double getSignalBoxAvg() { return m_signalPipeline.getSignalBoxAvg(); }
    public double getSignalBoxTopAvg() { return m_signalPipeline.getSignalTopAvg(); }
    public double getSignalBoxBottomAvg() { return m_signalPipeline.getSignalBottomAvg(); }
    public int getSignalPipelineFrameCount() { return m_signalPipeline.getFrameCount(); }

    // This method should only be called from TaskReadSignal. It sets the local Vision variable
    // (m_signal) for later use.
    public void ReadSignal(Alliance alliance, FieldSide fieldSide) {
        m_signal = m_signalPipeline.getSignal();
    }

    //============================================================================================
    // Camera Tilt Servo Functionality

    public void setCameraTiltForSignal() {
        m_hwVision.setCameraTilt(CAMERA_TILT_SIGNAL);
    }

    public void setCameraTiltForMedPole() {
        m_hwVision.setCameraTilt(CAMERA_TILT_MED_POLE);
    }

    public void setCameraTiltForHighPole() {
        m_hwVision.setCameraTilt(CAMERA_TILT_HIGH_POLE);
    }

    //============================================================================================

    public void reportData(Telemetry telemetry) {
        if (false && m_signalPipeline != null && m_findPolePipeline != null) {
//            telemetry.addData("Pole",
//                    "upper " + getPoleUpperCol() +
//                    " lower " + getPoleLowerCol() +
//                    " width " + getUpperPoleWidth());
//            logCsvString("Vision: Signal, " + getSignal() +
//                    " ");
        }
    }
}
