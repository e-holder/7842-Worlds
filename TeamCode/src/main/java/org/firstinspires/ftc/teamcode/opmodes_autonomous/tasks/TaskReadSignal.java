package org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks;

import org.firstinspires.ftc.teamcode.middleware.VisionPipelineSignal.Signal;

public class TaskReadSignal extends AutonomousTask {

    private enum TaskState {
        WAIT_FOR_NON_BLACK_IMAGES,
        WAIT_FOR_WEBCAM,
        READ_SIGNAL,
    }

    private TaskState m_state;
    private TaskState m_priorState = TaskState.READ_SIGNAL; //Anything but WAIT_FOR_NON_BLACK_IMAGES
    private int m_webcamWaitCounter = 0;
    private final boolean m_isTelemetryOn;
    private double m_avgBox;
    private double m_avgTop;
    private double m_avgBottom;

    public TaskReadSignal(boolean isTelemetryOn) {
        m_state = TaskState.WAIT_FOR_NON_BLACK_IMAGES;

        m_isTelemetryOn = isTelemetryOn;
    }

    public void displaySignalTelemetry() {
        if (m_isTelemetryOn) {
            telemetry.addData("Park = ", parkingZone +
                    ", Signal = " + signal);
        }
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
                // ONLY transition to RUN state after we know the pipeline is processing frames with
                // non-empty images.
                if (vera.vision.areSignalImagesValid()) {
                    vera.logCsvString("RS frames to non-black: " +
                            vera.vision.getSignalPipelineFrameCount());
                    m_state = TaskState.WAIT_FOR_WEBCAM;

                }
                break;
            case WAIT_FOR_WEBCAM:
                // Skip an image or two in case the first is sketchy.
                if (++m_webcamWaitCounter >= 2) {
                    m_state = TaskState.READ_SIGNAL;
                }
                break;
            case READ_SIGNAL:
                vera.vision.ReadSignal(alliance, fieldSide);
                signal = vera.vision.getSignal();
                parkingZone = (signal != Signal.UNKNOWN ? signal : Signal.ZONE3);
                vera.logCsvString("TaskReadSignal: signal, " + signal +
                        ", avgBox, " + df3.format(m_avgBox) +
                        ", avgTop, " + df3.format(m_avgTop) +
                        ", avgBottom, " + df3.format(m_avgBottom));
                taskStatus = TaskStatus.DONE;
                break;
        }
        return taskStatus;
    }
}
