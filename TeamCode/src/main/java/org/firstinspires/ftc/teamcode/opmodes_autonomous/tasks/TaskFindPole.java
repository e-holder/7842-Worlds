package org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks;

public class TaskFindPole extends AutonomousTask {

    private enum TaskState {
        INIT,
        FIND_POLE,
    }

    private TaskState m_state;
    private TaskState m_priorState = TaskState.FIND_POLE; // Anything but INIT

    public TaskFindPole() {
        m_state = TaskState.INIT;
    }

    public void setPoleType(PoleType poleType) {
        vera.vision.setPoleType(poleType);
    }

    public double getOffsetToPole_deg() {
        return (m_state == TaskState.FIND_POLE ? vera.vision.getDeltaToPole_deg() : 0.0);
    }

    public double getDistToScore_in() {
        return (m_state == TaskState.FIND_POLE ? vera.vision.getDistToScore_in() : 2.0);
    }

    @Override
    public TaskStatus update() {
        if (m_state != m_priorState) {
            m_priorState = m_state;
            vera.logCsvString("TaskFindPole state, " + m_state);
        }

        final int WAIT_FOR_NON_BLACK_FRAMES_COUNT = 500;
        TaskStatus taskStatus = TaskStatus.RUNNING;
        int frameNumber;

        switch (m_state) {
            case INIT:
                frameNumber = vera.vision.getFindPoleFrameCount();
                // ONLY transition to FIND_POLE state after we know the pipeline is processing
                // frames with non-black images.
                if (vera.vision.areFindPoleFramesValid()) {
                    vera.logCsvString(("FindPole non-black at frame: ") + frameNumber);
                    m_state = TaskState.FIND_POLE;
                } else if (frameNumber >= WAIT_FOR_NON_BLACK_FRAMES_COUNT) {
                    vera.logCsvString("FindPole frames BLACK! frame = " + frameNumber);
                    m_state = TaskState.FIND_POLE;
                }
                break;
            case FIND_POLE:
                // Nothing to do but allow the pipeline to run.
                break;
        }
        return taskStatus;
    }
}
