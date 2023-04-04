package org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks;

import org.firstinspires.ftc.teamcode.middleware.VisionPipelineSignal.Signal;

public class TaskReadSignal extends AutonomousTask {

    private enum TaskState {
        WAIT_FOR_NON_BLACK_IMAGES,
        READ_SIGNAL,
    }

    private final int WAIT_FOR_NON_BLACK_FRAMES_COUNT = 500;
    private final int SKIP_FRAMES_PRIOR_TO_USE = 3;

    private TaskState m_state;
    private TaskState m_priorState = TaskState.READ_SIGNAL; //Anything but WAIT_FOR_NON_BLACK_IMAGES
    private int m_frameNumber;
    private int m_priorFrameNumber = -1;
    private int m_minFrameNumber;
    private double m_avgBox;
    private double m_avgTop;
    private double m_avgBottom;

    public TaskReadSignal() {
        m_state = TaskState.WAIT_FOR_NON_BLACK_IMAGES;
    }

    public void addSignalTelemetry() {
        telemetry.addData("Park = ", parkingZone +
                ", Signal = " + signal);
        telemetry.addData("averages",
                (int)m_avgBox + " (" + (int)m_avgTop + " / " + (int)m_avgBottom + ")");
    }

    public Signal getParkingZone() {
        return parkingZone;
    }

    @Override
    public TaskStatus update() {
        if (m_state != m_priorState) {
            m_priorState = m_state;
            vera.logCsvString("ReadSignal state, " + m_state);
        }

        TaskStatus taskStatus = TaskStatus.RUNNING;

        m_avgBox = vera.vision.getSignalBoxAvg();
        m_avgTop = vera.vision.getSignalBoxTopAvg();
        m_avgBottom = vera.vision.getSignalBoxBottomAvg();

        switch (m_state) {
            case WAIT_FOR_NON_BLACK_IMAGES:
                m_frameNumber = vera.vision.getSignalPipelineFrameCount();
                m_minFrameNumber = m_frameNumber + SKIP_FRAMES_PRIOR_TO_USE;
                // Transition to RUN state after we know the pipeline is processing frames with
                // non-empty images (or until we give up).
                if (vera.vision.areSignalImagesValid()) {
                    vera.logCsvString("ReadSignal non-black at frame: " + m_frameNumber);
                    m_state = TaskState.READ_SIGNAL;
                } else if (m_frameNumber >= WAIT_FOR_NON_BLACK_FRAMES_COUNT) {
                    vera.logCsvString("ReadSignal frames are all BLACK! frame = " +
                            m_frameNumber);
                    m_state = TaskState.READ_SIGNAL;
                }
                break;
            case READ_SIGNAL:
                m_frameNumber = vera.vision.getSignalPipelineFrameCount();
                if (m_frameNumber != m_priorFrameNumber) {
                    m_priorFrameNumber = m_frameNumber;
                    vera.vision.ReadSignal(alliance, fieldSide);
                    signal = vera.vision.getSignal();
                    parkingZone = (signal != Signal.UNKNOWN ? signal : Signal.ZONE3);
                    vera.logCsvString("TaskReadSignal: signal, " + signal +
                            ", avgBox, " + df3.format(m_avgBox) +
                            ", avgTop, " + df3.format(m_avgTop) +
                            ", avgBottom, " + df3.format(m_avgBottom) +
                            ", frame, " + m_frameNumber);
                }
                if (m_frameNumber >= m_minFrameNumber) {
                    // Skip a few frames in case the first is sketchy.
                    taskStatus = TaskStatus.DONE;
                }
                break;
        }
        return taskStatus;
    }
}
