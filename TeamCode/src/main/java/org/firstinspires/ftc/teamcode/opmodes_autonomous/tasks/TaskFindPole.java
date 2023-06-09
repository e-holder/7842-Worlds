package org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.middleware.Vera;

public class TaskFindPole implements CONSTANTS {

    public enum TaskState {
        INIT,
        INITIALIZING,
        IDLE,
        FIND_POLE,
        SKIPPING_FRAMES,
        FINDING_POLE,
    }

    private Vera m_vera;
    private TaskState m_state;
    private TaskState m_priorState = TaskState.FIND_POLE; // Anything but INIT

    private int m_startLoopCount;
    private int m_frameCount;
    private int m_priorFrameCount;
    private int m_startFrameCount;;
    private int m_framesToSkip;

    public TaskFindPole(Vera vera) {
        m_vera = vera;
        m_state = TaskState.INIT;
    }

    public void setInitializationFindPoleMode(FindPoleMode findPoleMode) {
        m_vera.vision.setFindPoleMode(findPoleMode, "init");
    }

    public void startFindingPole(FindPoleMode findPoleMode, String coneName) {
        m_vera.vision.setFindPoleMode(findPoleMode, coneName);
        switch (m_state) {
            case INIT:
            case INITIALIZING:
            case IDLE:
                m_vera.vision.logFindPoleData("starting", 0);
                m_state = TaskState.FIND_POLE;
                break;
        }
    }

    public void stopFindingPole() {
        switch (m_state) {
            case FIND_POLE:
            case SKIPPING_FRAMES:
            case FINDING_POLE:
                m_vera.vision.logFindPoleData("stopping", 0);
                m_state = TaskState.IDLE;
        }
    }

    public boolean isFindingPole() {
        return (m_state != TaskState.IDLE);
    }

    public boolean isPoleDetected() {
        if (m_state == TaskState.FIND_POLE || m_state == TaskState.SKIPPING_FRAMES) {
            return true;
        } else {
            return m_vera.vision.isPoleDetected();
        }
    }

    public double getOffsetToPole_deg() {
        if (m_state == TaskState.FIND_POLE || m_state == TaskState.SKIPPING_FRAMES) {
            return 0.0;
        } else {
            return m_vera.vision.getDeltaToPole_deg();
        }
    }

    public double getDistToScore_in() {
        if (m_state == TaskState.FIND_POLE || m_state == TaskState.SKIPPING_FRAMES) {
            return 0.0;
        } else {
            return m_vera.vision.getDistToScore_in();
        }
    }

    public TaskState update() {
        if (m_state != m_priorState) {
            m_priorState = m_state;
            m_vera.logCsvString("TaskFindPole state, " + m_state);
        }

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
                m_priorFrameCount = m_frameCount - 1;
                m_startFrameCount = m_frameCount;
                m_startLoopCount = m_vera.getLoopCount();
                m_vera.vision.findPoleEnable(true);
                m_framesToSkip = 8;
                m_state = TaskState.SKIPPING_FRAMES;
                break;
            case SKIPPING_FRAMES:
                m_frameCount = m_vera.vision.getFindPoleFrameCount();
                if (m_frameCount > m_priorFrameCount) {
                    m_priorFrameCount = m_frameCount;
                    m_framesToSkip--;
                    if (m_vera.vision.isPoleDetected() ||
                            (m_frameCount - m_startFrameCount) > m_framesToSkip) {
                        m_vera.vision.logFindPoleData("endSkipping", 0);
                        m_state = TaskState.FINDING_POLE;
                    } else {
                        m_vera.vision.logFindPoleData("skipping", 0);
                    }
                }
                break;
            case FINDING_POLE:
                if (true || Vera.isVisionTestMode) {
                    m_frameCount = m_vera.vision.getFindPoleFrameCount();
                    if (m_frameCount > m_priorFrameCount) {
                        m_priorFrameCount = m_frameCount;
                        double loopsPerFrame = (double)(m_vera.getLoopCount() - m_startLoopCount) /
                                        Math.max(0.0001, (m_frameCount - m_startFrameCount));
                        m_vera.vision.logFindPoleData("finding", loopsPerFrame);
                    }
                }
                break;
        }
        return m_state;
    }
}