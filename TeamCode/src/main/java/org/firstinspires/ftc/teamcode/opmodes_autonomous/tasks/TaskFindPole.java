package org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.middleware.Vera;

public class TaskFindPole implements CONSTANTS {

    public enum TaskState {
        INIT,
        INITIALIZING,
        IDLE,
        FIND_POLE,
        FINDING_POLE,
    }

    private final int MAX_DETECTION_ATTEMPTS = 20;

    private Vera m_vera;
    private TaskState m_state;
    private TaskState m_priorState = TaskState.FIND_POLE; // Anything but INIT

    private boolean m_isPoleDetected = false;
    private double m_deltaToPole_deg;
    private double m_distToScore_in;
    private int m_startLoopCount;
    private int m_frameCount;
    private int m_startFrameCount;
    private int m_priorFrameCount;
//    private int m_lastLoggedFrameCount = -1;

    public TaskFindPole(Vera vera) {
        m_vera = vera;
        m_state = TaskState.INIT;
    }

    public void startFindingPole(PoleType poleType) {
        m_vera.vision.setPoleType(poleType);
        if (m_state != TaskState.FIND_POLE && m_state != TaskState.FINDING_POLE) {
            m_state = TaskState.FIND_POLE;
        }
    }

    public void stopFindingPole() {
        if(m_state != TaskState.INIT && m_state != TaskState.INITIALIZING) {
            m_state = TaskState.IDLE;
        }
    }

    public boolean isFindingPole() {
        return (m_state != TaskState.IDLE);
    }

    public boolean isPoleDetected() { return m_isPoleDetected; }
    public double getOffsetToPole_deg() {
        return m_deltaToPole_deg;
    }
    public double getDistToScore_in() {
        return m_distToScore_in;
    }

    public TaskState update() {
//        if (m_state != m_priorState) {
//            m_priorState = m_state;
//            m_vera.logCsvString("TaskFindPole state, " + m_state);
//        }

        final int WAIT_FOR_NON_BLACK_FRAMES_COUNT = 200;

        switch (m_state) {
            case INIT:
                if (!m_vera.vision.isFindPoleStreaming()) {
                    m_vera.vision.startStreaming(VeraPipelineType.FIND_POLE);
                }
                m_state = TaskState.INITIALIZING;
                break;
            case INITIALIZING:
                // INIT is complete if pipeline is processing frames with non-black images.
                m_frameCount = m_vera.vision.getFindPoleFrameCount();
                if (!m_vera.vision.isFindPolePipelineFrameBlack()) {
                    m_vera.logCsvString("FindPole non-black at frame: " + m_frameCount);
                    m_state = TaskState.IDLE;
                } else if (m_frameCount >= WAIT_FOR_NON_BLACK_FRAMES_COUNT) {
                    m_vera.logCsvString("FindPole frames BLACK! frame = " + m_frameCount);
                    m_state = TaskState.IDLE;
                }
                break;
            case IDLE:
                m_vera.vision.findPoleEnable(false);
                break;
            case FIND_POLE:
                m_frameCount = m_vera.vision.getFindPoleFrameCount();
                m_startFrameCount = m_frameCount;
                m_priorFrameCount = m_frameCount - 1;
                m_startLoopCount = m_vera.getLoopCount();
                m_vera.vision.findPoleEnable(true);
                m_state = TaskState.FINDING_POLE;
                break;
            case FINDING_POLE:
                m_frameCount = m_vera.vision.getFindPoleFrameCount();
                if (m_frameCount > m_priorFrameCount) {
                    m_priorFrameCount = m_frameCount;
                    m_isPoleDetected = m_vera.vision.isPoleDetected();
                    m_deltaToPole_deg = m_vera.vision.getDeltaToPole_deg();
                    m_distToScore_in = m_vera.vision.getDistToScore_in();
//                    if (m_frameCount != m_lastLoggedFrameCount) {
//                        double loopsPerFrame =
//                                (double) (m_vera.getLoopCount() - m_startLoopCount) /
//                                        Math.max(1.0, (m_frameCount - m_lastLoggedFrameCount));
//                        m_vera.vision.logFindPoleData("Detected", loopsPerFrame);
//                        m_lastLoggedFrameCount = m_frameCount;
//                    }
                    m_state = TaskState.IDLE;
                }
                break;
        }
        return m_state;
    }
}