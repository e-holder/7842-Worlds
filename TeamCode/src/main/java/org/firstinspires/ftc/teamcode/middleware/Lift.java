package org.firstinspires.ftc.teamcode.middleware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HwLift;

public class Lift implements CONSTANTS {

    private enum LiftState {
        INIT,
        REQUEST_MOVE_TO_BOTTOM,
        MOVE_TO_BOTTOM,
        MOVING_TO_BOTTOM,
        IDLE_AT_BOTTOM,
        REQUEST_MOVE_TO_LOW_POLE,
        MOVE_TO_LOW_POLE_POS,
        MOVING_TO_LOW_POLE_POS,
        REQUEST_MOVE_TO_MID_POLE,
        MOVE_TO_MID_POLE_POS,
        MOVING_TO_MID_POLE_POS,
        REQUEST_MOVE_TO_HIGH_POLE,
        MOVE_TO_HIGH_POLE_POS,
        MOVING_TO_HIGH_POLE_POS,
        DRIVER_PLACE_CONE,
        IDLE,
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

    private final int LIFT_AT_JUNCTION_TOL_TICK = 15;

    private final int CLAW_GRABBING_COUNT_MAX = 30;
    private final int CLAW_OPENING_COUNT_MAX = 30;

    private final int DELAY_FOR_GRAB = 12;
    private final int DELAY_FOR_MOVE_TO_BOTTOM = 10;
    private final int MOVE_TO_POLE_COUNT_MAX = 150;

    private final double CLAW_CLOSED = 0.2;
    private final double CLAW_OPEN = 0.67;

    private final double DRIVE_LIFT_TO_BOTTOM_DELTA_IN = 10.0;
    private final double RESET_POS_IN = -10.0;    // Drive into the stops FAST!
    private final double ENTERING_ROBOT_IN = 8.75; // Distance to drop cone on descent (safety)
    private final double LOW_POLE_IN = 12.0;
    private final double MID_POLE_IN = 24.0;
    private final double HIGH_POLE_IN = 38.5;

    private final double LIFT_DRIVER_PLACE_DELTA_IN = 0.5;
    private final double LIFT_BOTTOM_TOL_IN = 0.3;
    private final double LIFT_STALL_REDUCE_IN = 0.15;

    private final double LIFT_SPEED_FAST_TPS = 4500;
    private final double LIFT_SPEED_SLOW_TPS = 800.0;

    private final double LIFT_MAX_BOTTOM_IDLE_AMP = 0.6;

    private final double MIDDLEMAN_HAS_CONE_THRESH_IN = 2.0;

    // MEMBER DATA ================================================================================
    // SUBSYSTEM has an instance of its corresponding hardware class here.
    private HwLift m_hwLift;

    private StringBuilder m_csvLogStr = new StringBuilder();
    private boolean m_hasLiftBeenReset = false;
    private boolean m_isLimitPressed;
    private boolean m_isLiftBusy;
    private boolean m_isClawClosed;
    private boolean m_hasDriverBeenNotifiedOfCone = false;
    private int m_clawGrabbingCounter = CLAW_GRABBING_COUNT_MAX;
    private int m_clawOpeningCounter = CLAW_OPENING_COUNT_MAX;
    private int m_delayForGrabCounter;
    private int m_delayForMoveToBottom = 0;
    private int m_moveToPoleCounter;
    private int m_liftPos_tick;
    private int m_liftTarget_tick;
    private double m_liftTargetSpeed;
    private double m_liftTargetPos_in;
    private double m_liftTargetDelta_in;
    private double m_liftPos_in;
    private double m_liftMotorCurrent_amp;
    private double m_middlemanSensorDist_in;

    private Vera m_vera;
    private LiftState m_state;
    private LiftState m_priorState = LiftState.IDLE;  // Set to anything but INIT.
    private PlaceConeCommand m_placeConeCommand = PlaceConeCommand.STOP;

    // SUBSYSTEM has a public constructor here.
    public Lift(Vera vera) {
        m_vera = vera;

        // SUBSYSTEM gets its corresponding hardware class instance here.
        m_hwLift = vera.getHwLift();

        if (m_vera.isAutonomous()) {
            closeClaw();  // Grabs pre-load cone for autonomous startup.
        }
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
        m_hasLiftBeenReset = true;
    }

    private void moveLiftToTargetPositionAtTargetSpeed() {
        // This entire if structure is safety code to keep the motor from pushing the lift
        // relentlessly against the bottom stops.
        if (m_isLimitPressed) {
            switch (m_state) {
                case IDLE:
                case IDLE_AT_BOTTOM:
                case MOVING_TO_BOTTOM:
                case DRIVER_PLACE_CONE:
                    m_liftTargetPos_in = LIFT_BOTTOM_TOL_IN;
                    m_liftTargetSpeed = 0.0;
                    break;
            }
        } else if (m_state == LiftState.IDLE_AT_BOTTOM) {
            if (m_liftPos_in <= LIFT_BOTTOM_TOL_IN) {
                // If we are close enough, stay where we are.
                m_liftTargetPos_in = LIFT_BOTTOM_TOL_IN;
                m_liftTargetSpeed = 0.0;
            } else if (m_liftMotorCurrent_amp > LIFT_MAX_BOTTOM_IDLE_AMP) {
                // If the motor is stalling, start raising the target position.
                m_liftTargetPos_in += LIFT_STALL_REDUCE_IN;
            } else {
                // Keep trying to drive to the trigger switch.
                m_liftTargetPos_in = 0.0;
                m_liftTargetSpeed = LIFT_SPEED_FAST_TPS;
            }
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

        // Prevent a grabbed cone from being driven back into the robot.
        if (m_liftTargetPos_in < m_liftPos_in && m_liftPos_in < ENTERING_ROBOT_IN) {
            openClaw();
        }
    }

    private void closeClaw() {
        m_hwLift.setLiftClawPos(CLAW_CLOSED);
    }

    private void openClaw() {
        if (!m_vera.isAutonomous() || m_hasLiftBeenReset) {
            m_hwLift.setLiftClawPos(CLAW_OPEN);
        }
    }

    // ===================== PUBLIC FUNCTIONS ==============================

    public void logCsvString(String record) { m_csvLogStr.append(record).append("\n"); }

    public StringBuilder getLogString() {
        return m_csvLogStr;
    }

    public void getInputs() {
        m_isLiftBusy = m_hwLift.isLiftBusy();
        m_liftPos_tick = m_hwLift.getLiftPosition_ticks();
        m_liftPos_in = m_hwLift.getLiftPosition_in();
        m_liftMotorCurrent_amp = m_hwLift.getLiftMotorCurrent_amp();
        m_isLimitPressed = m_hwLift.isLimitSwitchPressed();
        m_middlemanSensorDist_in = m_hwLift.getMiddlemanSensorDistance_in();

        // NOTE: getLiftClawPos only reflects what the servo has been commanded. It does not
        //  necessarily reflect where the claw servo actually is.
        m_isClawClosed = (m_hwLift.getLiftClawPos() <= (CLAW_CLOSED + 0.1));
    }

    public boolean hasMiddlemanReceivedCone() {
        boolean rv = false;
        boolean hasCone = m_middlemanSensorDist_in < MIDDLEMAN_HAS_CONE_THRESH_IN;
        if (hasCone && !m_hasDriverBeenNotifiedOfCone) {
            rv = true;
            m_hasDriverBeenNotifiedOfCone = true;
        } else if (m_state == LiftState.DRIVER_PLACE_CONE) {
            m_hasDriverBeenNotifiedOfCone = false;
        }
        return rv;
    }

    public void driverPlaceConeCommand(double cmd) {
        m_liftTargetDelta_in = Math.abs(cmd) * LIFT_DRIVER_PLACE_DELTA_IN;
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

    public void moveLiftToMidPole() {
        m_state = LiftState.REQUEST_MOVE_TO_MID_POLE;
    }

    public void moveLiftToHighPole() {
        m_state = LiftState.REQUEST_MOVE_TO_HIGH_POLE;
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

    public void update() {
        // Always keep this logging block.
        if (m_priorState != m_state) {
            m_priorState = m_state;
            logCsvString("state, " + m_state);
        }

        switch (m_state) {
            case INIT:
                m_delayForMoveToBottom = 0;
                m_state = LiftState.MOVE_TO_BOTTOM;
                break;
            case IDLE_AT_BOTTOM:  // Intentional fall-through
            case IDLE:
                // Nothing to do
                break;
            case REQUEST_MOVE_TO_BOTTOM:
                openClaw();
                m_delayForMoveToBottom = (m_liftPos_in <= LOW_POLE_IN ?
                        DELAY_FOR_MOVE_TO_BOTTOM : 0);
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
                if (m_isLimitPressed) {
                    resetLift();
                    m_state = LiftState.IDLE_AT_BOTTOM;
                } else if (!m_isLiftBusy) {
                    // If we finish moving down, but haven't hit the bottom, move down more.
                    m_liftTargetPos_in = m_liftPos_in - DRIVE_LIFT_TO_BOTTOM_DELTA_IN;
                    m_liftTargetSpeed = LIFT_SPEED_FAST_TPS;
                }
                break;
            case MOVING_TO_LOW_POLE_POS:   // Intentional fall-through
            case MOVING_TO_MID_POLE_POS:   // Intentional fall-through
            case MOVING_TO_HIGH_POLE_POS:
                m_moveToPoleCounter++;
                double delta = Math.abs(m_liftTarget_tick - m_liftPos_tick);
                if (!m_isLiftBusy || delta <= LIFT_AT_JUNCTION_TOL_TICK ||
                        m_moveToPoleCounter > MOVE_TO_POLE_COUNT_MAX) {
                    m_state = LiftState.DRIVER_PLACE_CONE;
                }
                break;
            case REQUEST_MOVE_TO_LOW_POLE:
                closeClaw();
                m_delayForGrabCounter = m_liftPos_in < ENTERING_ROBOT_IN ? DELAY_FOR_GRAB : 0;
                m_state = LiftState.MOVE_TO_LOW_POLE_POS;
                break;
            case MOVE_TO_LOW_POLE_POS:
                closeClaw();
                m_delayForGrabCounter--;
                if (m_delayForGrabCounter <= 0) {
                    m_moveToPoleCounter = 0;
                    m_liftTargetPos_in = LOW_POLE_IN;
                    m_liftTargetSpeed = LIFT_SPEED_FAST_TPS;
                    m_state = LiftState.MOVING_TO_LOW_POLE_POS;
                }
                break;
            case REQUEST_MOVE_TO_MID_POLE:
                closeClaw();
                m_delayForGrabCounter = m_liftPos_in < ENTERING_ROBOT_IN ? DELAY_FOR_GRAB : 0;
                m_state = LiftState.MOVE_TO_MID_POLE_POS;
                break;
            case MOVE_TO_MID_POLE_POS:
                closeClaw();
                m_delayForGrabCounter--;
                if (m_delayForGrabCounter <= 0) {
                    m_moveToPoleCounter = 0;
                    m_liftTargetPos_in = MID_POLE_IN;
                    m_liftTargetSpeed = LIFT_SPEED_FAST_TPS;
                    m_state = LiftState.MOVING_TO_MID_POLE_POS;
                }
                break;
            case REQUEST_MOVE_TO_HIGH_POLE:
                closeClaw();
                m_delayForGrabCounter = m_liftPos_in < ENTERING_ROBOT_IN ? DELAY_FOR_GRAB : 0;
                m_state = LiftState.MOVE_TO_HIGH_POLE_POS;
                break;
            case MOVE_TO_HIGH_POLE_POS:
                closeClaw();
                m_delayForGrabCounter--;
                if (m_delayForGrabCounter <= 0) {
                    m_moveToPoleCounter = 0;
                    m_liftTargetPos_in = HIGH_POLE_IN;
                    m_liftTargetSpeed = LIFT_SPEED_FAST_TPS;
                    m_state = LiftState.MOVING_TO_HIGH_POLE_POS;
                }
                break;
            case DRIVER_PLACE_CONE:
                driverControlConePlacement();
                if (m_isLimitPressed) {
                    resetLift();
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
        if (false) {
            logCsvString("lift" +
                    ", isLimit, " + m_isLimitPressed +
                    ", amp, " + df3.format(m_liftMotorCurrent_amp) +
                    ", tgtIn, " + df3.format(m_liftTargetPos_in) +
                    ", posIn, " + df3.format(m_liftPos_in) +
//                    ", tgtTick, " + m_liftTarget_tick +
//                    ", posTick, " + m_liftPos_tick +
                    ", tgtSpeed, " + m_liftTargetSpeed +
//                    ", tgtDelta, " + df3.format(m_liftTargetDelta_in) +
//                    ", placeCmd, " + m_placeConeCommand +
//                    ", isBusy, " + m_isLiftBusy +
                    ", isClawClosed, " + m_isClawClosed +
                    ", grabDelay, " + m_delayForGrabCounter +
//                    ", grabCount, " + m_clawGrabbingCounter +
                    ", openCount, " + m_clawOpeningCounter +
//                    ", downDelay, " + m_delayForMoveToBottom +
//                    ", poleCount, " + m_moveToPoleCounter +
                    ".");
        }

        if (false) {
            telemetry.addData("Lift",
//                    "hasCone = " + isConeInMiddleman() +
//                    "coneDist_in = " + df3.format(m_middlemanSensorDist_in) +
                    "");
        }
    }
}
