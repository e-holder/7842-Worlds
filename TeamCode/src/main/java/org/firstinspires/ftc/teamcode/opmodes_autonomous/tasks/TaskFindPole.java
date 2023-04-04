package org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks;

import static org.firstinspires.ftc.teamcode.middleware.Vision.NOMINAL_HIGH_POLE_WIDTH_PIX;
import static org.firstinspires.ftc.teamcode.middleware.Vision.NOMINAL_MID_POLE_WIDTH_PIX;

public class TaskFindPole extends AutonomousTask {

    private enum TaskState {
        INIT,
        WAIT_FOR_NON_BLACK_IMAGES,
        FIND_POLE,
    }

    private final int WAIT_FOR_NON_BLACK_FRAMES_COUNT = 500;

    private TaskState m_state;
    private TaskState m_priorState = TaskState.FIND_POLE; // Anything but INIT

    private final PoleType m_poleType;

    private int m_frameNumber;
    private int m_priorFrameNumber = -1;
    private int m_upperPoleDelta_pix;
    private int m_lowerPoleDelta_pix;
    private int m_poleWidth_pix;

    public TaskFindPole(PoleType poleType) {
        switch (poleType) {
            case HIGH:
                m_poleType = PoleType.HIGH;
                break;
            case MID:
                m_poleType = PoleType.MID;
                break;
            default: // We do not use the lift for low poles, so this case is irrelevant really.
                m_poleType = PoleType.LOW;
                break;
        }

        m_state = TaskState.INIT;
    }

    public void addFindPoleTelemetry() {
        telemetry.addData("Pole ",
                "upper " + m_upperPoleDelta_pix +
                        " lower " + m_lowerPoleDelta_pix +
                        " width " + m_poleWidth_pix);
        telemetry.update();
    }

    private final double STRAFE_DIST_PER_PIX_UPPER_HIGH_IN = 1.0 / 60.0; // 1 inch per X pixels
    private final double STRAFE_DIST_PER_PIX_HIGH_LEAN_IN = 1.0 / 15.0;   // 1 inch per X pixels
    private final double STRAFE_DIST_HIGH_MAX_IN = 2.5;

    private double getStrafeDistToHighPole_in() {
        double lean_pix = m_upperPoleDelta_pix - m_lowerPoleDelta_pix;
        double strafeDist_in = Math.min(STRAFE_DIST_HIGH_MAX_IN,
                m_upperPoleDelta_pix * STRAFE_DIST_PER_PIX_UPPER_HIGH_IN +
                        lean_pix * STRAFE_DIST_PER_PIX_HIGH_LEAN_IN);
        strafeDist_in = Math.max(strafeDist_in, -STRAFE_DIST_HIGH_MAX_IN);
        return strafeDist_in;
    }

    private final double STRAFE_DIST_PER_PIX_UPPER_MID_IN = 1.0 / 80.0; // 1 inch per X pixels
    private final double STRAFE_DIST_PER_PIX_MID_LEAN_IN = 1.0 / 30.0;  // 1 inch per X pixels
    private final double STRAFE_DIST_MID_MAX_IN = 3.0;

    private double getStrafeDistToMidPole_in() {
        double lean_pix = m_upperPoleDelta_pix - m_lowerPoleDelta_pix;
        double strafeDist_in = Math.min(STRAFE_DIST_MID_MAX_IN,
                m_upperPoleDelta_pix * STRAFE_DIST_PER_PIX_UPPER_MID_IN +
                        lean_pix * STRAFE_DIST_PER_PIX_MID_LEAN_IN);
        strafeDist_in = Math.max(strafeDist_in, -STRAFE_DIST_MID_MAX_IN);
        return strafeDist_in;
    }

    public double getLateralStrafeToPole_in() {
        switch (m_poleType) {
            case HIGH:
                return getStrafeDistToHighPole_in();
            case MID:
            default:
                return getStrafeDistToMidPole_in();
        }
    }

    private final double DIST_PER_PIX_DELTA_WIDTH_HIGH_IN = 1.0 / 10.0; // 1 inch per X pixels
    private final double DIST_PER_PIX_DELTA_WIDTH_MID_IN = 1.0 / 20.0;  // 1 inch per X pixels
    private final double PLACE_DIST_MAX_IN = 3.5;

    public double getScoreDistToPole_in() {
        double scoreDist_in = 0.5 * PLACE_DIST_MAX_IN;
        switch (m_poleType) {
            case HIGH:
                scoreDist_in += ((NOMINAL_HIGH_POLE_WIDTH_PIX - m_poleWidth_pix) *
                        DIST_PER_PIX_DELTA_WIDTH_HIGH_IN);
                break;
            default:
                scoreDist_in += ((NOMINAL_MID_POLE_WIDTH_PIX - m_poleWidth_pix) *
                        DIST_PER_PIX_DELTA_WIDTH_MID_IN);
                break;
        }
        return scoreDist_in;
    }

    @Override
    public TaskStatus update() {
        if (m_state != m_priorState) {
            m_priorState = m_state;
            vera.logCsvString("TVFP state, " + m_state);
        }

        TaskStatus taskStatus = TaskStatus.RUNNING;

        switch (m_state) {
            case INIT:
                vera.vision.setPoleType(m_poleType);
                vera.logCsvString("setPole, " + m_poleType);
                m_state = TaskState.WAIT_FOR_NON_BLACK_IMAGES;
                break;
            case WAIT_FOR_NON_BLACK_IMAGES:
                m_frameNumber = vera.vision.getFindPolePipelineFrameCount();
                // ONLY transition to RUN state after we know the pipeline is processing frames
                // with non-empty images.
                if (vera.vision.areFindPoleImagesValid()) {
                    vera.logCsvString(("FindPole non-black at frame: ") + m_frameNumber);
                    m_state = TaskState.FIND_POLE;
                } else if (m_frameNumber >= WAIT_FOR_NON_BLACK_FRAMES_COUNT) {
                    vera.logCsvString("FindPole frames are all BLACK! frame = " +
                            m_frameNumber);
                    m_state = TaskState.FIND_POLE;
                }
                break;
            case FIND_POLE:
                m_frameNumber = vera.vision.getSignalPipelineFrameCount();
                if (m_frameNumber != m_priorFrameNumber) {
                    m_priorFrameNumber = m_frameNumber;
                    m_upperPoleDelta_pix = vera.vision.getPoleUpperOffset_pix();
                    m_lowerPoleDelta_pix = vera.vision.getPoleLowerOffset_pix();
                    m_poleWidth_pix = vera.vision.getUpperPoleWidth_pix();
                    vera.logCsvString("TaskFindPole" +
                            ", upper, " + m_upperPoleDelta_pix +
                            ", lower, " + m_lowerPoleDelta_pix +
                            ", width, " + m_poleWidth_pix);
                }
                break;
        }
        return taskStatus;
    }
}
