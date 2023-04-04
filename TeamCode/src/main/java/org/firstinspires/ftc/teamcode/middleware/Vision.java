package org.firstinspires.ftc.teamcode.middleware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HwVision;
import org.openftc.easyopencv.OpenCvPipeline;

public class Vision implements CONSTANTS {

    private final double CAMERA_TILT_SIGNAL = 0.0;
    // For the pole vision camera tilt, the following settings need to allow us to see just the pole
    // in our detection box without seeing any of a 4-stack on cones on that pole.
    private final double CAMERA_TILT_HIGH_POLE = 0.285;    // About 12 degrees up
    private final double CAMERA_TILT_MID_POLE = 0.33;      // About 30 degrees up

    // SUBSYSTEM has an instance of its corresponding hardware class here.
    private HwVision m_hwVision;

    private StringBuilder m_csvLogStr = new StringBuilder();
    private final PoleType m_defaultPoleType;

    // SUBSYSTEM has a public constructor here.
    public Vision(Vera vera, Telemetry telemetry, PoleType defaultPoleType) {
        m_defaultPoleType = defaultPoleType;

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
                if (m_defaultPoleType == PoleType.HIGH) {
                    setCameraTiltForHighPole();
                } else {
                    setCameraTiltForMidPole();
                }
                pipeline = m_findPolePipeline;
                setMinPoleWidth();
                break;
            case SIGNAL:   // Intentional fall-through
            default:
                setCameraTiltForSignal();
                pipeline = m_signalPipeline;
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
    }

    //============================================================================================
    // Find Pole Vision Functionality

    // Camera will be about 19 inches (18-20) from High poles
    // Camera will be about 4 inches above the floor and about 45 degrees up from horizontal
    // The pipeline processing box should sit right on top of a stack of 4 cones on the pole.
    public static final int NOMINAL_HIGH_POLE_CENTER_PIX = VisionPipelineFindPole.BOX_WIDTH/2 + 48;
    public static final int NOMINAL_HIGH_POLE_WIDTH_PIX = 32;
    private final int MIN_HIGH_POLE_WIDTH_PIX = 24;

    // Camera will be about 10 inches (9-11) from Mid poles
    // Camera will be about 4 inches above the floor and about 30 degrees up from horizontal
    // The pipeline processing box should sit right on top of a stack of 4 cones on the pole.
    public static final int NOMINAL_MID_POLE_CENTER_PIX = VisionPipelineFindPole.BOX_WIDTH/2 + 51;
    public static final int NOMINAL_MID_POLE_WIDTH_PIX = 54;
    private final int MIN_MID_POLE_WIDTH_PIX = 37;

    // TODO: Calibrate low pole constants (if we ever need them in autonomous)
    // Camera will be about 2.5 inches (2-4) from Low poles.
    public static final int NOMINAL_LOW_POLE_CENTER_PIX = VisionPipelineFindPole.BOX_WIDTH/2 - 4;
    public static final int NOMINAL_LOW_POLE_WIDTH_PIX = 160;
    private final int MIN_LOW_POLE_WIDTH_PIX = 100;

    // This is public to allow test autonomous OpModes to set the pole type.
    public static PoleType poleType = PoleType.MID;

    private VisionPipelineFindPole m_findPolePipeline;

    public void setPoleType(PoleType newPoleType) {
        poleType = newPoleType;
        setMinPoleWidth();
        switch (newPoleType) {
            case HIGH:
                setCameraTiltForHighPole();
                break;
            case MID:
            default:
                setCameraTiltForMidPole();
                break;
        }
    }

    public void setMinPoleWidth() {
        switch (poleType) {
            case HIGH:
                m_findPolePipeline.setMinPoleWidth(MIN_HIGH_POLE_WIDTH_PIX);
                break;
            case MID:
                m_findPolePipeline.setMinPoleWidth(MIN_MID_POLE_WIDTH_PIX);
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
            case HIGH:
                offset_pix -= NOMINAL_HIGH_POLE_CENTER_PIX;
                break;
            case MID:
                offset_pix -= NOMINAL_MID_POLE_CENTER_PIX;
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
            case HIGH:
                offset_pix -= NOMINAL_HIGH_POLE_CENTER_PIX;
                break;
            case MID:
                offset_pix -= NOMINAL_MID_POLE_CENTER_PIX;
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
            case HIGH:
                nominalWidth_pix = NOMINAL_HIGH_POLE_WIDTH_PIX;
                break;
            case MID:
                nominalWidth_pix = NOMINAL_MID_POLE_WIDTH_PIX;
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

    public void setCameraTiltForMidPole() {
        m_hwVision.setCameraTilt(CAMERA_TILT_MID_POLE);
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
