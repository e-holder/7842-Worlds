package org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks;

import org.firstinspires.ftc.teamcode.middleware.VisionPipelineSignal.Signal;

public class TaskReadSignal extends AutonomousTask {

    public enum TaskState {
        WAIT_FOR_NON_BLACK_IMAGES,
        READ_SIGNAL,
        SIGNAL_OKAY,
        DONE
    }

    private TaskState m_state;
    private TaskState m_priorState = TaskState.READ_SIGNAL; //Anything but WAIT_FOR_NON_BLACK_IMAGES
    private int m_minFrameNumber;

    public TaskReadSignal() {
        m_state = TaskState.WAIT_FOR_NON_BLACK_IMAGES;
    }

    public void stopSignalVision() {
        m_state = TaskState.DONE;
    }

    // Do not call this method until the task status is TASK_DONE.
    public Signal getParkingZone() {
        return vera.vision.getParkingZone();
    }

    public TaskState getState() { return m_state; }

    public TaskState update(boolean readFinal) {
        if (m_state != m_priorState) {
            m_priorState = m_state;
            vera.logCsvString("ReadSignal state, " + m_state);
        }

        final int WAIT_FOR_NON_BLACK_FRAMES_COUNT = 500;
        final int SKIP_FRAMES_PRIOR_TO_USE = 3;
        TaskStatus taskStatus = TaskStatus.RUNNING;
        int frameNumber;

        switch (m_state) {
            case WAIT_FOR_NON_BLACK_IMAGES:
                frameNumber = vera.vision.getSignalPipelineFrameCount();
                // Transition to READ_SIGNAL state after we know the pipeline is processing frames
                // with non-black images (or until we give up).
                if (!vera.vision.isSignalPipelineFrameBlack()) {
                    vera.logCsvString("ReadSignal non-black at frame: " + frameNumber);
                    m_minFrameNumber = frameNumber + SKIP_FRAMES_PRIOR_TO_USE;
                    m_state = TaskState.READ_SIGNAL;
                } else if (frameNumber >= WAIT_FOR_NON_BLACK_FRAMES_COUNT) {
                    vera.logCsvString("ReadSignal frames BLACK! frame = " + frameNumber);
                    m_minFrameNumber = frameNumber + SKIP_FRAMES_PRIOR_TO_USE;
                    m_state = TaskState.READ_SIGNAL;
                }
                break;
            case READ_SIGNAL:
                frameNumber = vera.vision.getSignalPipelineFrameCount();
                if (frameNumber >= m_minFrameNumber) {
                    // Skip a few frames in case the first is sketchy.
                    m_state = TaskState.SIGNAL_OKAY;
                }
                break;
            case SIGNAL_OKAY:
                if (readFinal) {
                    m_state = TaskState.DONE;
                }
                break;
            case DONE:
                break;
        }
        return m_state;
    }
}
