package org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;

public class TaskTestVisionFindPole extends AutonomousTask {

    private enum TaskState {
        INIT,
        WAIT_FOR_NON_BLACK_IMAGES,
        FIND_POLE,
    }

    private TaskState m_state;
    private TaskState m_priorState = TaskState.FIND_POLE; // Anything but INIT

    private final CornerType m_poleType;
    private final boolean m_isTelemetryOn;

    public TaskTestVisionFindPole(CornerType poleType, boolean isTelemetryOn) {
        switch (poleType) {
            case H:
                m_poleType = CornerType.H;
                vera.vision.setCameraTiltForHighPole();
                break;
            case M:
                m_poleType = CornerType.M;
                vera.vision.setCameraTiltForMedPole();
                break;
            default: // We do not use the lift for low poles, so this case is irrelevant really.
                m_poleType = CornerType.L;
                vera.vision.setCameraTiltForMedPole();
                break;
        }
        m_isTelemetryOn = isTelemetryOn;

        m_state = TaskState.INIT;
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
                m_state = TaskState.WAIT_FOR_NON_BLACK_IMAGES;
                break;
            case WAIT_FOR_NON_BLACK_IMAGES:
                // ONLY transition to RUN state after we know the pipeline is processing frames
                // with non-empty images.
                if (vera.vision.areFindPoleImagesValid()) {
                    m_state = TaskState.FIND_POLE;
                }
                break;
            case FIND_POLE:
                int upperPoleDelta_pix = vera.vision.getPoleUpperOffset_pix();
                int lowerPoleDelta_pix = vera.vision.getPoleLowerOffset_pix();
                int poleWidth_pix = vera.vision.getUpperPoleWidth_pix();
                if (m_isTelemetryOn) {
                    telemetry.addData("Pole ",
                            "upper " + upperPoleDelta_pix +
                                    " lower " + lowerPoleDelta_pix +
                                    " width " + poleWidth_pix);
                    telemetry.update();
                }
                break;
        }
        return taskStatus;
    }
}
