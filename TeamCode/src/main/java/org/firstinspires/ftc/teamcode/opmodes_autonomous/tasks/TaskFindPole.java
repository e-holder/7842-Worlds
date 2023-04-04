package org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks;

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
