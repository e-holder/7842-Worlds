package org.firstinspires.ftc.teamcode.middleware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HwLift;

public class Lift implements CONSTANTS {

    private enum LiftState {
        INIT,
        IDLE,
        REQUEST_MOVE_TO_BOTTOM,
        MOVE_TO_BOTTOM,
        MOVING_TO_BOTTOM,
        REQUEST_MOVE_TO_LOW_POLE,
        MOVE_TO_LOW_POLE_POS,
        MOVING_TO_LOW_POLE_POS,
        REQUEST_MOVE_TO_MED_POLE,
        MOVE_TO_MED_POLE_POS,
        MOVING_TO_MED_POLE_POS,
        REQUEST_MOVE_TO_HIGH_POLE,
        MOVE_TO_HIGH_POLE_POS,
        MOVING_TO_HIGH_POLE_POS,
        DRIVER_PLACE_CONE,
        GRAB_CONE,
        GRABBING_CONE,
        DROP_CONE,
        DROPPING_CONE,
    }

    // Supports finer-grained control to move the lift for placing a cone under driver control.
    public enum PlaceConeCommand {
        STOP,
        UP_SLOW,
        DOWN_SLOW
    }

    private final int LIFT_AT_BOTTOM_TOL_TICK = 3;

    private final int CLAW_GRABBING_COUNT_MAX = 30;
    private final int CLAW_OPENING_COUNT_MAX = 30;

    private final int DELAY_FOR_GRAB_COUNT = 3;
    private final int DELAY_FOR_MOVE_TO_BOTTOM = 10;
    private final int MOVE_TO_POLE_COUNT_MAX = 150;

    private final double CLAW_CLOSED = 0.2;
    private final double CLAW_OPEN = 0.67;

    private final double DRIVE_LIFT_TO_BOTTOM_DELTA_IN = 10.0;
    private final double RESET_POS_IN = -10.0;    // Drive into the stops FAST!
    private final double ENTERING_ROBOT_IN = 5.0; // Distance to drop cone on descent (safety)
    private final double LOW_POLE_IN = 12.0;
    private final double MED_POLE_IN = 24.0;
    private final double HIGH_POLE_IN = 39.5;

    private final double LIFT_MOTOR_DOWN_STALL_AMP = 5.0;
    private final double LIFT_SPEED_FAST_TPS = 4500;
    private final double LIFT_SPEED_SLOW_TPS = 800.0;
    private final double LIFT_TARGET_DELTA_IN = 0.5;

    // MEMBER DATA ================================================================================
    // SUBSYSTEM has an instance of its corresponding hardware class here.
    private HwLift m_hwLift;

    private StringBuilder m_csvLogStr = new StringBuilder();
    private boolean m_hasLiftBeenReset = false;
    private boolean m_isLiftMotorStalled;
    private boolean m_isLiftAtBottom;
    private boolean m_isLiftBusy;
    private boolean m_isClawClosed;
    private int m_clawGrabbingCounter = CLAW_GRABBING_COUNT_MAX;
    private int m_clawOpeningCounter = CLAW_OPENING_COUNT_MAX;
    private int m_delayForGrabCounter;
    private int m_delayForMoveToBottom;
    private int m_moveToPoleCounter;
    private int m_liftPos_tick;
    private int m_liftTarget_tick;
    private double m_liftTargetSpeed;
    private double m_liftTargetPos_in;
    private double m_liftTargetDelta_in;
    private double m_liftPos_in;
    private double m_liftMotor_amp;

    private LiftState m_state;
    private LiftState m_priorState = LiftState.IDLE;  // Set to anything but INIT.
    private PlaceConeCommand m_placeConeCommand = PlaceConeCommand.STOP;

    // SUBSYSTEM has a public constructor here.
    public Lift(Vera vera) {
        // SUBSYSTEM gets its corresponding hardware class instance here.
        m_hwLift = vera.getHwLift();

        m_state = LiftState.INIT;
    }

    //  SUBSYSTEM has a private constructor here.
    private Lift() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Default constructor not supported.");
    }

    // ===================== PRIVATE METHODS ==============================

    private void resetLift() {
        m_hwLift.resetLiftMotor(m_liftPos_tick);
        m_liftTargetPos_in = 0.0;
        openClaw();
        m_hasLiftBeenReset = true;
    }

    private void stopLift() {
        m_liftTargetPos_in = 0.0;
    }

    private void moveLiftToTargetPositionAtTargetSpeed() {
        // Ensure cone is dropped if this move might drive a held cone into the robot and damage it.
        if (m_liftTargetPos_in < m_liftPos_in  && m_liftTargetPos_in < ENTERING_ROBOT_IN) {
            openClaw();
        }
        m_liftTarget_tick = m_hwLift.liftRunToPosition(m_liftTargetPos_in, m_liftTargetSpeed);
    }

    private void driverControlConePlacement() {
        double pos_in = m_liftTargetPos_in;
        if (m_placeConeCommand == PlaceConeCommand.UP_SLOW) {
            pos_in += m_liftTargetDelta_in;
        } else if (m_placeConeCommand == PlaceConeCommand.DOWN_SLOW) {
            pos_in -= m_liftTargetDelta_in;
        }
        m_liftTargetPos_in = pos_in;
        m_liftTargetSpeed = LIFT_SPEED_SLOW_TPS;
    }

    private void closeClaw() {
        m_hwLift.setLiftClawPos(CLAW_CLOSED);
    }

    private void openClaw() {
        m_hwLift.setLiftClawPos(CLAW_OPEN);
    }

    // ===================== PUBLIC FUNCTIONS ==============================

    public void logCsvString(String record) { m_csvLogStr.append(record).append("\n"); }

    public StringBuilder getLogString() {
        return m_csvLogStr;
    }

    public void getInputs() {
        m_liftMotor_amp = m_hwLift.getLiftMotorCurrent_amp();
        m_isLiftBusy = m_hwLift.isLiftBusy();
        m_liftPos_tick = m_hwLift.getLiftPosition_ticks();
        m_liftPos_in = m_hwLift.getLiftPosition_in();

        m_isLiftMotorStalled = (m_liftMotor_amp >= LIFT_MOTOR_DOWN_STALL_AMP);
        m_isLiftAtBottom = m_isLiftMotorStalled ||
                (m_hasLiftBeenReset && m_liftPos_tick <= LIFT_AT_BOTTOM_TOL_TICK);
        // NOTE: getLiftClawPos only reflects what the servo has been commanded. It does not
        //  necessarily reflect where the claw servo actually is.
        m_isClawClosed = (m_hwLift.getLiftClawPos() <= (CLAW_CLOSED + 0.1));
    }

    public void driverPlaceConeCommand(double cmd) {
        m_liftTargetDelta_in = Math.abs(cmd) * LIFT_TARGET_DELTA_IN;
        if (cmd > 0.05) {
            m_placeConeCommand = PlaceConeCommand.UP_SLOW;
        } else if (cmd < -0.05) {
            m_placeConeCommand = PlaceConeCommand.DOWN_SLOW;
        }
        else {
            m_placeConeCommand = PlaceConeCommand.STOP;
            m_liftTargetDelta_in = 0.0;
        }
    }

    public void moveLiftToBottom() {
        m_state = LiftState.REQUEST_MOVE_TO_BOTTOM;
    }

    public void moveLiftToLowPole() {
        m_state = LiftState.REQUEST_MOVE_TO_LOW_POLE;
    }

    public void moveLiftToMediumPole() {
        m_state = LiftState.REQUEST_MOVE_TO_MED_POLE;
    }

    public void moveLiftToHighPole() {
        m_state = LiftState.REQUEST_MOVE_TO_HIGH_POLE;
    }

    public boolean isBusy() {
        return !(m_state == LiftState.IDLE);
    }

    // This is really just a driver manual override command. It should only happen if the driver
    // commands it in an odd situation.
    public void grabCone() {
        m_state = LiftState.GRAB_CONE;
    }

    public void dropCone() {
        m_state = LiftState.DROP_CONE;
    }

    public void toggleClawOverride() {
        if (m_isClawClosed) {
            dropCone();
        } else {
            grabCone();
        }
    }

    public void commandComponents () {
        // Always keep this logging block.
        if (true && (m_priorState != m_state)) {
            m_priorState = m_state;
            logCsvString("state, " + m_state);
        }

        switch (m_state) {
            case INIT:
                m_state = LiftState.MOVE_TO_BOTTOM;
                break;
            case IDLE:
                break;
            case REQUEST_MOVE_TO_BOTTOM:
                if (m_liftPos_in <= 10.0) {
                    m_delayForMoveToBottom = (m_liftPos_in < MED_POLE_IN ?
                            DELAY_FOR_MOVE_TO_BOTTOM : 0);
                }
                m_state = LiftState.MOVE_TO_BOTTOM;
                break;
            case MOVE_TO_BOTTOM:
                if (m_delayForMoveToBottom > 0) {
                    m_delayForMoveToBottom--;
                } else {
                    m_liftTargetPos_in = RESET_POS_IN;
                    m_liftTargetSpeed = LIFT_SPEED_FAST_TPS;
                    openClaw();
                    m_state = LiftState.MOVING_TO_BOTTOM;
                }
                break;
            case MOVING_TO_BOTTOM:
                if (!m_hasLiftBeenReset && m_isLiftAtBottom) {
                    resetLift();
                    m_state = LiftState.IDLE;
                } else if (m_isLiftAtBottom) {
                    stopLift();
                    m_state = LiftState.IDLE;
                } else if (!m_isLiftBusy) {
                    // If we finish moving down, but haven't hit the bottom, move down more.
                    m_liftTargetPos_in = m_liftPos_in - DRIVE_LIFT_TO_BOTTOM_DELTA_IN;
                    m_liftTargetSpeed = LIFT_SPEED_FAST_TPS;
                }
                break;
            case MOVING_TO_LOW_POLE_POS:   // Intentional fall-through
            case MOVING_TO_MED_POLE_POS:   // Intentional fall-through
            case MOVING_TO_HIGH_POLE_POS:
                m_moveToPoleCounter++;
                if (!m_isLiftBusy || m_moveToPoleCounter > MOVE_TO_POLE_COUNT_MAX) {
                    m_state = LiftState.DRIVER_PLACE_CONE;
                }
                break;
            case REQUEST_MOVE_TO_LOW_POLE:
                closeClaw();
                m_delayForGrabCounter = DELAY_FOR_GRAB_COUNT;
                m_state = LiftState.MOVE_TO_LOW_POLE_POS;
                break;
            case MOVE_TO_LOW_POLE_POS:
                closeClaw();
                if (m_delayForGrabCounter-- <= 0) {
                    m_moveToPoleCounter = 0;
                    m_liftTargetPos_in = LOW_POLE_IN;
                    m_liftTargetSpeed = LIFT_SPEED_FAST_TPS;
                    m_state = LiftState.MOVING_TO_LOW_POLE_POS;
                }
                break;
            case REQUEST_MOVE_TO_MED_POLE:
                closeClaw();
                m_delayForGrabCounter = DELAY_FOR_GRAB_COUNT;
                m_state = LiftState.MOVE_TO_MED_POLE_POS;
                break;
            case MOVE_TO_MED_POLE_POS:
                closeClaw();
                if (m_delayForGrabCounter-- <= 0) {
                    m_moveToPoleCounter = 0;
                    m_liftTargetPos_in = MED_POLE_IN;
                    m_liftTargetSpeed = LIFT_SPEED_FAST_TPS;
                    m_state = LiftState.MOVING_TO_MED_POLE_POS;
                }
                break;
            case REQUEST_MOVE_TO_HIGH_POLE:
                closeClaw();
                m_delayForGrabCounter = DELAY_FOR_GRAB_COUNT;
                m_state = LiftState.MOVE_TO_HIGH_POLE_POS;
                break;
            case MOVE_TO_HIGH_POLE_POS:
                closeClaw();
                if (m_delayForGrabCounter-- <= 0) {
                    m_moveToPoleCounter = 0;
                    m_liftTargetPos_in = HIGH_POLE_IN;
                    m_liftTargetSpeed = LIFT_SPEED_FAST_TPS;
                    m_state = LiftState.MOVING_TO_HIGH_POLE_POS;
                }
                break;
            case DRIVER_PLACE_CONE:
                driverControlConePlacement();
                if (m_isLiftAtBottom) {
                    stopLift();
                    m_state = LiftState.IDLE;
                }
                break;
            case DROP_CONE:
                openClaw();
                m_clawOpeningCounter = 0;
                m_state = LiftState.DROPPING_CONE;
                break;
            case DROPPING_CONE:
                m_clawOpeningCounter++;
                if (m_clawOpeningCounter >= CLAW_OPENING_COUNT_MAX) {
                    m_state = LiftState.IDLE;
                }
                break;
            case GRAB_CONE:
                // This is really just a driver manual override state. It should only happen if
                // the driver commands it in an odd situation.
                closeClaw();
                m_clawGrabbingCounter = 0;
                m_state = LiftState.GRABBING_CONE;
                break;
            case GRABBING_CONE:
                m_clawGrabbingCounter++;
                if (m_clawGrabbingCounter >= CLAW_GRABBING_COUNT_MAX) {
                    m_state = LiftState.IDLE;
                }
                break;
        }

        moveLiftToTargetPositionAtTargetSpeed();
    }

    public void reportData(Telemetry telemetry) {
        if (true) {
            logCsvString("lift" +
                    ", motorAmp, " + df3.format(m_liftMotor_amp) +
//                    ", isBottom, " + m_isLiftAtBottom +
//                    ", posTick, " + m_liftPos_tick +
                    ", pos, " + df3.format(m_liftPos_in) +
//                    ", targetTick, " + m_liftTarget_tick +
                    ", targetPos, " + df3.format(m_liftTargetPos_in) +
//                    ", tgtDelta, " + df3.format(m_liftTargetDelta_in) +
//                    ", placeCmd, " + m_placeConeCommand +
//                    ", isBusy, " + m_isLiftBusy +
//                    ", isClawClosed, " + m_isClawClosed +
                    ", grabDelay, " + m_delayForGrabCounter +
                    ", grabCount, " + m_clawGrabbingCounter +
                    ", downDelay, " + m_delayForMoveToBottom +
//                    ", openCount, " + m_clawOpeningCounter +
//                    ", poleCount, " + m_moveToPoleCounter +
                    ".");
        }

//        This is useful for calibrating inches per motor tick.
//        if (true) {
//            telemetry.addData("Lift",
//                    " ticks " + m_liftPos_ticks +
//                    " inches " + df3.format(m_liftPos_in));
//        }
    }
}
