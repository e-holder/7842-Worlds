package org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;

import java.util.concurrent.TimeUnit;

public class TaskDelay extends AutonomousTask {

    private enum TaskState {
        START_TIMER,
        WAIT_FOR_TIMER_TO_EXPIRE,
        DONE
    }

    private TaskState m_state;
    private ElapsedTime m_delayTimer = new ElapsedTime();
    private long m_delay_ms;

    public TaskDelay(long delay_ms) {
        m_state = (delay_ms == 0 ? TaskState.DONE : TaskState.START_TIMER);

        m_delay_ms = delay_ms;
    }

    private TaskDelay() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Default constructor not supported.");
    }

    @Override
    public CONSTANTS.TaskStatus update(double runTime_s) {
        CONSTANTS.TaskStatus taskStatus = CONSTANTS.TaskStatus.RUNNING;
        switch (m_state) {
            case START_TIMER:
                m_delayTimer.reset();
                m_state = TaskState.WAIT_FOR_TIMER_TO_EXPIRE;
                break;
            case WAIT_FOR_TIMER_TO_EXPIRE:
                if (m_delayTimer.time(TimeUnit.MILLISECONDS) >= m_delay_ms) {
                    m_state = TaskState.DONE;
                }
                break;
            case DONE:
                taskStatus = CONSTANTS.TaskStatus.DONE;
                break;
        }
        return taskStatus;
    }
}
