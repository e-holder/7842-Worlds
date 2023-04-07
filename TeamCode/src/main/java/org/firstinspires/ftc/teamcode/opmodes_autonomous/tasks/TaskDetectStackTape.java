package org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks;

public class TaskDetectStackTape extends AutonomousTask {

    private enum TaskState {
        INIT,
        COLLECT_DATA,
    }

    private TaskState m_state;
    private TaskState m_priorState = TaskState.COLLECT_DATA; //Anything but INIT

    public TaskDetectStackTape() {
        m_state = TaskState.INIT;
    }

    // Do not call this method until the task status is TASK_DONE.
//    public Signal getParkingZone() {
//        return vera.vision.getParkingZone();
//    }

    @Override
    public TaskStatus update() {
        if (m_state != m_priorState) {
            m_priorState = m_state;
            vera.logCsvString("DetectStackTape state, " + m_state);
        }

        TaskStatus taskStatus = TaskStatus.RUNNING;

        switch (m_state) {
            case INIT:
                vera.intake.enableStackTapeCalibrationMode();
                vera.intake.turnOnStackTapeSensing();
                m_state = TaskState.COLLECT_DATA;
                break;
            case COLLECT_DATA:
                taskStatus = TaskStatus.DONE;
                break;
        }
        return taskStatus;
    }
}
