package org.firstinspires.ftc.teamcode.middleware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HwIntake;

import java.util.List;

public class Intake implements CONSTANTS {

    private enum IntakeState {
        INIT_DELAY,
        INIT,
        MOVE_TO_RESET_POS,   // Fully "inside" the robot (past Eject position)
        MOVING_TO_RESET_POS,
        DRIVER_CONE_INTAKE,
        MOVE_TO_EJECT_POS,
        MOVING_TO_EJECT_POS,
        MOVE_TO_CONE_POS,
        MOVING_TO_CONE_POS,
        MOVE_TO_BEACON_POS,
        MOVING_TO_BEACON_POS,
        MOVE_TO_BEACON_PLACE_POS,
        MOVING_TO_BEACON_PLACE_POS,
        EJECT_CONE,
        EJECTING_CONE,
        HOLD_AT_LOW_JUNCTION_POS,
        MOVE_TO_IDLE_POS,    // Out of the way of the lift, poles, other bots (near vertical).
        MOVING_TO_IDLE_POS,
        IDLE,
    }

    public enum IntakeConeCommand {
        STOP,
        DOWN_SLOW,
        UP_SLOW
    }

    private final int DEFAULT_AUTONOMOUS_INIT_DELAY_COUNT = 10;

    private final double ARM_RESET_DEG = -33.0;
    private final double ARM_EJECT_DEG = ARM_RESET_DEG;
    private final double ARM_IDLE_DEG = -10.0;
    private final double ARM_LOW_JUNCTION_DEG = 30.0;
    private final double ARM_FAST_RESET_POINT_DEG = 50.0;  // If arm is further than this, go fast
    private final double ARM_BEACON_PLACE_DEG = 52.0;
    private final double ARM_CONE5_DEG = 85.0;
    private final double ARM_CONE4_DEG = 94.0;
    private final double ARM_CONE3_DEG = 102.0;
    private final double ARM_CONE2_DEG = 110.0;
    private final double ARM_BEACON_DEG = 104.0;
    private final double ARM_CONE1_DEG = 112.0;
    private final double ARM_MAX_DEG = 112.0;    // Note: Max physical position is about 112.

    private final double ARM_ARRIVAL_TOLERANCE_DEG = 2.0;
    private final double ARM_TARGET_DELTA_DEG = 1.0;
    private final double ARM_SLOW_RESET_DELTA_DEG = 10.0;  // Drive arm slowly if it is near reset
    private final double ARM_FAST_RESET_DELTA_DEG = 30.0;  // Drive arm fast if it is far from reset
    private final double ARM_DELTA_CONESTACK_WRIST_DELAY_DEG = 5.0;

    private final double ARM_SPEED_FAST = 3500;  // Note: Increases appear to end around 3500..4500.
    private final double ARM_SPEED_SLOW = 2500;  // Used for initial reset, and driver control
    private final double ARM_SPEED_SLOW_EJECT = 2000; // Avoids disturbing cone stack.
    private final double ARM_DRIVER_CONTROL_CMD_SCALE = 20.0;

    private final double WRIST_POS_AT_AUTONOMOUS_SHUTDOWN_DEG = 0.0;
    private final double WRIST_POS_EJECT_CONE_DEG = 15.0;
    private final double WRIST_POS_AT_LOW_JUNCTION_DEG = 160.0;
    private final double WRIST_POS_IDLE_DEG = 170.0;

    private final double WRIST_POS_STACK_DELTA_DEG = 135.0;
    private final double WRIST_POS_BEACON_DELTA_DEG = 90.0;
    private final double WRIST_POS_CONE_DELTA_DEG = 137.0;

    private final double INTAKE_WHEELS_STALL_AMP = 7.0;
    private final double DEFAULT_INTAKE_WHEEL_SPEED = 1.0;
    private final double DEFAULT_INTAKE_WHEEL_EJECT_SPEED = -1.0;
    private final double INTAKE_CONE_HOLD_WHEEL_SPEED = 0.1;
    private final int EJECT_CONESTACK_DELAY_COUNT = 3;
    private final int EJECT_COUNT = 10;

    //                             LEFT     RIGHT
    // BLUE:         full tape:  360-445   372-480
    // BLUE:       center tape:  233-289   223-354
    // BLUE: 2-inch off center:  193-167   164-232
    // BLUE:           no tape:  139-147   144-158
    private final double STACK_TAPE_THRESH = 160;  // TODO: Calibrate for RED too (this is BLUE)

    // MEMBER DATA ================================================================================
    // SUBSYSTEM has an instance of its corresponding hardware class here.
    private HwIntake m_hwIntake;
    private Vera m_vera;

    private IntakeState m_state;
    private IntakeState m_priorState = IntakeState.IDLE;  // Anything but INIT
    private IntakeConeCommand m_intakeConeCommand = IntakeConeCommand.STOP;
    private boolean m_isLimitSwitchPressed;
    private boolean m_isArmBusy;
    private boolean m_isStackTapeCalibrationMode = false;
    private boolean m_isStackTapeSensingOn = false;
    private boolean m_hasResetOccurred = false;
    private boolean m_areIntakeWheelsStalled;
    private boolean m_hasCone;
    private boolean m_hasConeOverride;
    private boolean m_isConeStackMode = false;
    private boolean m_isBeaconMode = false;
    private boolean m_isLowJunctionMode = false;
    private boolean m_autonomousShutdown = false;
    private double[][] m_stackTapeData = new double[500][3];  // 0 = left, 1 = right, 2 = Y pos
    private double m_leftVal, m_rightVal, m_poseY_in;
    private double m_priorPosY_in = -999.0;
    private double m_wristServoPos;
    private double m_wristPos_deg;  // 0 is inside/parallel to arm, 180 is extended/parallel to arm.
    private double m_wristCmdPos_deg = 0.0;
    private double m_armPos_deg;    // 0 is vertical, 90 is extended/front parallel to floor.
    private double m_armTargetPos_deg;
    private double m_armPosAtHasCone_deg;
    private double m_armTargetSpeed;
    private double m_intakeArmMotor_amp;
    private double m_intakeWheelMotor_amp;
    private double m_intakeWheelSpeed;
    private double m_intakeOverrideWheelSpeed;
    private double m_armDelta_deg = ARM_TARGET_DELTA_DEG;
    private double m_armDriverCmd;
    private int m_autonomousInitDelayCount = DEFAULT_AUTONOMOUS_INIT_DELAY_COUNT;
    private int m_initDelayCounter = 0;
    private int m_ejectDelayCounter = 0;
    private int m_ejectCounter;
    private int m_targetConeStackLevel;
    private int m_armPos_ticks;
    private int m_stackDataIdx = -1;
    private StringBuilder m_csvLogStr = new StringBuilder();

    // SUBSYSTEM has a public constructor here.
    public Intake(Vera vera) {
        // SUBSYSTEM gets its corresponding hardware class instance here.
        m_hwIntake = vera.getHwIntake();

        m_vera = vera;
        m_isLimitSwitchPressed = false;
        // This wrist move is required so our robot is shorter than 18" after INIT.
        m_hwIntake.wristMoveToPosition(10.0);
        m_state = IntakeState.INIT_DELAY;
    }

    //  SUBSYSTEM has a private constructor here.
    private Intake() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Default constructor not supported.");
    }

    // ============ PRIVATE METHODS===============

    private void resetArmMotor() {
        m_hwIntake.resetArmMotor(m_armPos_ticks);
        m_hasResetOccurred = true;
    }

    private void moveArmToTargetPositionAtTargetSpeed() {
        m_hwIntake.armRunToPosition(m_armTargetPos_deg, m_armTargetSpeed);
    }

    private void moveToTargetConeStackLevel() {
        double armTarget_deg;
        switch (m_targetConeStackLevel) {
            case 5:
                armTarget_deg = ARM_CONE5_DEG;
                break;
            case 4:
                armTarget_deg = ARM_CONE4_DEG;
                break;
            case 3:
                armTarget_deg = ARM_CONE3_DEG;
                break;
            case 2:
                armTarget_deg = ARM_CONE2_DEG;
                break;
            case 1:
            default:
                armTarget_deg = ARM_CONE1_DEG;
                break;
        }
        m_armTargetPos_deg = armTarget_deg;
        m_armTargetSpeed = ARM_SPEED_FAST;
    }

    private void moveToReset() {
        // Note: The encoder has not been reset, so the current angle report is totally
        // unreliable. Start blindly moving toward the limit switch until we reach it.
        double armTarget_deg = m_armPos_deg - ARM_SLOW_RESET_DELTA_DEG;
        double armSpeed = ARM_SPEED_SLOW;
        // If we have a LONG way to go, move faster.
        if (m_armPos_deg > ARM_FAST_RESET_POINT_DEG) {
            armTarget_deg = m_armPos_deg - ARM_FAST_RESET_DELTA_DEG;
            armSpeed = ARM_SPEED_FAST;
        }
        m_armTargetPos_deg = armTarget_deg;
        m_armTargetSpeed = armSpeed;
    }

    private void controlDriverConeIntake() {
        if (m_isConeStackMode) {
            moveToTargetConeStackLevel();
        } else {
            // Move the arm slowly per driver commands. The default is to STOP the arm
            // motion by running the arm to its CURRENT position.
            double armTarget_deg = m_armPos_deg;
            if (m_intakeConeCommand != IntakeConeCommand.STOP) {
                armTarget_deg += m_armDelta_deg;
                armTarget_deg = Math.min(armTarget_deg, ARM_MAX_DEG);
                armTarget_deg = Math.max(armTarget_deg, ARM_LOW_JUNCTION_DEG);
            }
            m_armTargetPos_deg = armTarget_deg;
            m_armTargetSpeed = ARM_SPEED_FAST;
        }
    }

    private double computeWristAngle_deg() {
        double wristPos_deg = 0.0;
        if (m_hasResetOccurred) {
            if (m_isConeStackMode) {
                wristPos_deg = m_armPos_deg + WRIST_POS_STACK_DELTA_DEG;
                switch (m_state) {
                    case MOVING_TO_EJECT_POS:
                    case EJECT_CONE:
                    case EJECTING_CONE:
                        if ((m_armPosAtHasCone_deg - m_armPos_deg) >
                                ARM_DELTA_CONESTACK_WRIST_DELAY_DEG) {
                            wristPos_deg = WRIST_POS_EJECT_CONE_DEG;
                        }
                        break;
                }
            } else if (m_isBeaconMode) {
                wristPos_deg = m_armPos_deg - WRIST_POS_BEACON_DELTA_DEG;
            } else if (m_isLowJunctionMode && m_hasCone) {
                wristPos_deg = WRIST_POS_AT_LOW_JUNCTION_DEG;
            } else if (m_autonomousShutdown) {
                wristPos_deg = WRIST_POS_AT_AUTONOMOUS_SHUTDOWN_DEG;
            } else if (m_armTargetPos_deg <= ARM_EJECT_DEG) {
                wristPos_deg = WRIST_POS_EJECT_CONE_DEG;
            } else if (m_armTargetPos_deg <= ARM_CONE5_DEG) {
                wristPos_deg = WRIST_POS_IDLE_DEG;
            } else {
                // Note: At arm 107, we want wrist 223 (about 43-45 degrees up from arm).
                wristPos_deg = m_armPos_deg + WRIST_POS_CONE_DELTA_DEG;
            }
        }
        return wristPos_deg;
    }

    private void setWristPosition() {
        if (m_state != IntakeState.INIT_DELAY) {
            m_wristCmdPos_deg = computeWristAngle_deg();
            m_hwIntake.wristMoveToPosition(m_wristCmdPos_deg);
        }
    }

    private void setIntakeWheelSpeed() {
        double speed;
        if (m_intakeOverrideWheelSpeed != 0.0) {
            speed = m_intakeOverrideWheelSpeed;
        } else {
            speed = m_intakeWheelSpeed;
        }
        m_hwIntake.setIntakeWheelSpeed(speed);
        if (speed < 0.0) {
            m_hasCone = false;
        }
    }

    private void getStackTapeSensorInputs() {
        if (m_isStackTapeCalibrationMode) {
            m_poseY_in = getStackTapeCalibrationPositionY();
        } else {
            m_poseY_in = Math.abs(m_vera.drivetrain.getPoseEstimate().getY());
        }
        if (m_poseY_in > (m_priorPosY_in + 0.5)) {
            if (m_vera.alliance == Alliance.RED) {
                m_leftVal = m_hwIntake.getLeftTapeSensorRed();
                m_rightVal = m_hwIntake.getRightTapeSensorRed();
            } else {
                m_leftVal = m_hwIntake.getLeftTapeSensorBlue();
                m_rightVal = m_hwIntake.getRightTapeSensorBlue();
            }
            if (m_leftVal >= STACK_TAPE_THRESH || m_rightVal >= STACK_TAPE_THRESH) {
                if (m_stackDataIdx < 499) {
                    m_stackDataIdx++;
                }
                m_stackTapeData[m_stackDataIdx][0] = m_leftVal;
                m_stackTapeData[m_stackDataIdx][1] = m_rightVal;
                m_stackTapeData[m_stackDataIdx][2] = m_poseY_in;
                m_priorPosY_in = m_poseY_in;
            }
        }
    }

    // This is only to be used for Stack Tape Calibration opmode.
    private double getStackTapeCalibrationPositionY() {
        List<Double> wheelPositions = m_vera.drivetrain.getWheelPositions();
        return (0.25 * (wheelPositions.get(0) + wheelPositions.get(1) +
                wheelPositions.get(2) + wheelPositions.get(3)));
    }

    // ============ PUBLIC METHODS===============

    public void logCsvString(String record) {
        m_csvLogStr.append(record).append("\n");
    }

    public StringBuilder getLogString() {
        return m_csvLogStr;
    }

    public void getInputs() {
        m_isLimitSwitchPressed = m_hwIntake.isLimitSwitchPressed();
        m_intakeArmMotor_amp = m_hwIntake.getIntakeArmMotorCurrent_amp();
        m_intakeWheelMotor_amp = m_hwIntake.getIntakeWheelMotorCurrent_amp();
        m_armPos_deg = m_hwIntake.getArmPosition_deg();
        m_armPos_ticks = m_hwIntake.getArmPosition_ticks();
        // Note: These two inputs really just reflect the most recent command sent to the servo.
        //       They do not really tell you where the wrist is right now.
        m_wristServoPos = m_hwIntake.getWristServoPos();
        m_wristPos_deg = m_hwIntake.getWristPos_deg(m_wristServoPos);

        m_areIntakeWheelsStalled = (m_intakeWheelMotor_amp > INTAKE_WHEELS_STALL_AMP);
        m_hasCone |= (m_areIntakeWheelsStalled || m_hasConeOverride);
        m_isArmBusy = m_hwIntake.isArmBusy() &&
                (Math.abs(m_armTargetPos_deg - m_armPos_deg) > ARM_ARRIVAL_TOLERANCE_DEG);

        if (m_isStackTapeCalibrationMode && m_isStackTapeSensingOn) {
            getStackTapeSensorInputs();
        }
    }

    public void hasConeOverride(double stick) {
        m_hasConeOverride = (stick < -0.5);
        m_hasCone |= m_hasConeOverride;
    }

    public void moveToIntakeConePos(int coneLevel) {
        m_targetConeStackLevel = coneLevel;
        m_state = IntakeState.MOVE_TO_CONE_POS;
        m_isConeStackMode = (coneLevel > 1);
        m_isBeaconMode = false;
        // NOTE: Low Junction Mode is LEGAL here.
    }

    public void moveToIdlePos() {
        m_state = IntakeState.MOVE_TO_IDLE_POS;
        m_isConeStackMode = false;
        m_isLowJunctionMode = false;
        m_isBeaconMode = false;
    }

    public void moveToBeaconPlacePos() {
        m_state = IntakeState.MOVE_TO_BEACON_PLACE_POS;
        m_isConeStackMode = false;
        m_isLowJunctionMode = false;
        m_isBeaconMode = true;
    }

    public void toggleBeaconMode() {
        m_isBeaconMode = !m_isBeaconMode;

        if (m_isBeaconMode) {
            m_isConeStackMode = false;
            m_isLowJunctionMode = false;
            m_state = IntakeState.MOVE_TO_BEACON_POS;
        } else {
            moveToIdlePos();
        }
    }

    public boolean toggleLowJunctionMode() {
        m_isLowJunctionMode = !m_isLowJunctionMode;
        m_isBeaconMode = false;
        // NOTE: ConeStackMode is LEGAL here
        return m_isLowJunctionMode;
    }

    public void moveWristParallelToArm() {
        m_autonomousShutdown = true;
    }

    public void setAutonomousInitDelayCount(int delayCount) {
        m_autonomousInitDelayCount = delayCount;
    }

    public void commandDriverConeIntake(double leftTrigger, double rightTrigger) {
        if (m_isConeStackMode) {
            m_armDriverCmd = rightTrigger;
            m_targetConeStackLevel = 5;            // Trigger released = 5
            if (m_armDriverCmd > 0.67) {
                m_targetConeStackLevel = 2;        // Trigger ~fully pressed = 2
            } else if (m_armDriverCmd > 0.33) {
                m_targetConeStackLevel = 3;        // Trigger ~halfway pressed = 3
            } else if (m_armDriverCmd > 0.01) {
                m_targetConeStackLevel = 4;        // Trigger pressed a little = 4
            }
        } else {
            m_armDriverCmd = rightTrigger - leftTrigger;
            m_armDelta_deg = m_armDriverCmd * ARM_DRIVER_CONTROL_CMD_SCALE;
            if (m_armDriverCmd > 0.01) {
                m_intakeConeCommand = IntakeConeCommand.DOWN_SLOW;
            } else if (m_armDriverCmd < -0.01) {
                m_intakeConeCommand = IntakeConeCommand.UP_SLOW;
            } else {
                m_intakeConeCommand = IntakeConeCommand.STOP;
                m_armDelta_deg = 0.0;
            }
        }
    }

    // This was originally coded to allow precise control of the intake wheel speed using the
    // left and right gamepad triggers. It mixes the two inputs (forward and reverse).
    public void setPreciseIntakeSpeed(double rightTriggerForward, double leftTriggerReverse) {
        m_intakeWheelSpeed = Math.max(-1.0, rightTriggerForward - leftTriggerReverse);
        m_intakeWheelSpeed = Math.min(1.0, m_intakeWheelSpeed);
    }

    public void setIntakeOverrideSpeed(double speed) {
        m_intakeOverrideWheelSpeed = speed;
    }

    public void moveToEjectPos() {
        m_state = IntakeState.MOVE_TO_EJECT_POS;
    }

    public void enableStackTapeCalibrationMode() {
        m_isStackTapeCalibrationMode = true;
    }

    public void turnOnStackTapeSensing() {
        m_isStackTapeSensingOn = true;
        m_stackDataIdx = -1;
        if (m_isStackTapeCalibrationMode) {
            m_vera.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            List<Double> wheelPositions = m_vera.drivetrain.getWheelPositions();
            m_priorPosY_in = -0.5 + getStackTapeCalibrationPositionY();
        } else {
            m_priorPosY_in = -999.0;
        }
    }

    public void turnOffStackTapeSensing() {
        m_isStackTapeSensingOn = false;
        logCsvString("Intake: stack tape detections");
        for (int idx = 0; idx <= m_stackDataIdx; idx++) {
            logCsvString("left, " + df3.format(m_stackTapeData[idx][0]) +
                    ", right, " + df3.format(m_stackTapeData[idx][1]) +
                    ", posY, " + df3.format(m_stackTapeData[idx][2]));
        }
    }

    public double getStackDeltaX_in() {
        // TODO: Implement
        double stackDeltaX_in = 0.0;
        logCsvString("Intake Stack deltaX, " + df3.format(stackDeltaX_in));
        return stackDeltaX_in;
    }

    public double getStackDeltaHeading_deg() {
        // TODO: Implement
        double stackDeltaHeading_deg = 0.0;
        logCsvString("Intake Stack deltaDeg, " + df3.format(stackDeltaHeading_deg));
        return stackDeltaHeading_deg;
    }

    public void update() {
        if (m_state != m_priorState) {
            m_priorState = m_state;
            logCsvString("state, " + m_state);
        }

        switch (m_state) {
            case INIT_DELAY:
                m_initDelayCounter++;
                if (!m_vera.isAutonomous() ||
                        m_initDelayCounter > m_autonomousInitDelayCount) {
                    m_state = IntakeState.INIT;
                }
                break;
            case INIT:
                if (m_isLimitSwitchPressed) {
                    resetArmMotor();
                    m_state = IntakeState.MOVE_TO_IDLE_POS;
                } else {
                    m_state = IntakeState.MOVE_TO_RESET_POS;
                }
                break;
            case MOVE_TO_RESET_POS:
                moveToReset();
                m_state = IntakeState.MOVING_TO_RESET_POS;
                break;
            case MOVING_TO_RESET_POS:
                if (m_isLimitSwitchPressed) {
                    resetArmMotor();
                    m_state = IntakeState.MOVE_TO_IDLE_POS;
                } else if (!m_isArmBusy) {
                    // Earlier move(s) didn't make it all the way. Keep moving toward limit switch.
                    moveToReset();
                }
                break;
            case MOVE_TO_EJECT_POS:
                m_armTargetPos_deg = ARM_EJECT_DEG;
                if (m_isConeStackMode) {
                    m_armTargetSpeed = ARM_SPEED_SLOW_EJECT;
                } else {
                    m_armTargetSpeed = ARM_SPEED_FAST;
                }
                m_state = IntakeState.MOVING_TO_EJECT_POS;
                break;
            case MOVING_TO_EJECT_POS:
                if (!m_isArmBusy) {
                    m_ejectDelayCounter = (m_isConeStackMode ? EJECT_CONESTACK_DELAY_COUNT : 0);
                    m_state = IntakeState.EJECT_CONE;
                }
                break;
            case MOVE_TO_BEACON_POS:
                m_armTargetPos_deg = ARM_BEACON_DEG;
                m_armTargetSpeed = ARM_SPEED_FAST;
                m_state = IntakeState.MOVING_TO_BEACON_POS;
                break;
            case MOVE_TO_BEACON_PLACE_POS:
                m_armTargetPos_deg = ARM_BEACON_PLACE_DEG;
                m_armTargetSpeed = ARM_SPEED_FAST;
                m_state = IntakeState.MOVING_TO_BEACON_PLACE_POS;
                break;
            case MOVING_TO_BEACON_PLACE_POS:  // Intentional fall-through
            case MOVING_TO_BEACON_POS:
                if (!m_isArmBusy) {
                    m_state = IntakeState.DRIVER_CONE_INTAKE;
                }
                break;
            case MOVE_TO_CONE_POS:
                moveToTargetConeStackLevel();
                m_state = IntakeState.MOVING_TO_CONE_POS;
                break;
            case MOVING_TO_CONE_POS:
                if (m_armPos_deg > ARM_FAST_RESET_POINT_DEG) {
                    m_intakeWheelSpeed = DEFAULT_INTAKE_WHEEL_SPEED;
                }
                if (!m_isArmBusy) {
                    m_state = IntakeState.DRIVER_CONE_INTAKE;
                }
                break;
            case DRIVER_CONE_INTAKE:
                controlDriverConeIntake();
                if (m_isLimitSwitchPressed) {
                    resetArmMotor();
                    m_state = IntakeState.MOVE_TO_IDLE_POS;
                } else if (m_isLowJunctionMode && m_hasCone) {
                    m_intakeWheelSpeed = INTAKE_CONE_HOLD_WHEEL_SPEED;
                    m_state = IntakeState.HOLD_AT_LOW_JUNCTION_POS;
                } else if (m_hasCone && !m_isBeaconMode) {
                    m_armPosAtHasCone_deg = m_armPos_deg;
                    m_intakeWheelSpeed = INTAKE_CONE_HOLD_WHEEL_SPEED;
                    m_state = IntakeState.MOVE_TO_EJECT_POS;
                }
                break;
            case EJECT_CONE:
                m_ejectDelayCounter--;
                if (m_ejectDelayCounter <= 0) {
                    if (m_hasCone) {
                        m_intakeWheelSpeed = DEFAULT_INTAKE_WHEEL_EJECT_SPEED;
                        m_ejectCounter = EJECT_COUNT;
                        m_state = IntakeState.EJECTING_CONE;
                    } else {
                        m_state = IntakeState.MOVE_TO_IDLE_POS;
                    }
                }
                break;
            case EJECTING_CONE:
                m_ejectCounter--;
                if (m_ejectCounter <= 0) {
                    m_intakeWheelSpeed = 0.0;
                    m_hasCone = false;
                    m_isConeStackMode = false;
                    m_state = IntakeState.MOVE_TO_IDLE_POS;
                }
                break;
            case HOLD_AT_LOW_JUNCTION_POS:
                m_armTargetPos_deg = ARM_LOW_JUNCTION_DEG;
                m_armTargetSpeed = ARM_SPEED_FAST;
                break;
            case MOVE_TO_IDLE_POS:
                m_armTargetPos_deg = ARM_IDLE_DEG;
                m_armTargetSpeed = ARM_SPEED_FAST;
                m_intakeWheelSpeed = 0.0;
                m_state = IntakeState.MOVING_TO_IDLE_POS;
                break;
            case MOVING_TO_IDLE_POS:
                if (!m_isArmBusy) {
                    m_state = IntakeState.IDLE;
                }
                break;
            case IDLE:
                m_intakeWheelSpeed = 0.0;
                break;
        }

        setWristPosition();
        moveArmToTargetPositionAtTargetSpeed();
        setIntakeWheelSpeed();
    }

    public void reportData(Telemetry telemetry) {

        if (false) {
            logCsvString("intake" +
//                    ", armAmp, " + df3.format(m_intakeArmMotor_amp) +
//                    ", wheelAmp, " + df3.format(m_intakeWheelMotor_amp) +
//                    ", hasCone, " + m_hasCone +
//                    ", armTgt, " + df3.format(m_armTargetPos_deg) +
//                    ", armDeg, " + df3.format(m_armPos_deg) +
//                    ", armTicks, " + m_armPos_ticks +
//                    ", armSpeed, " + m_armTargetSpeed +
//                    ", armBusy, " + m_isArmBusy +
//                    ", cmdDelta, " + df3.format(m_armDelta_deg) +
//                    ", coneCmd, " + m_intakeConeCommand +
//                    ", armCmd," + df3.format(m_armDriverCmd) +
//                    ", wristCmd, " + df3.format(m_wristCmdPos_deg) +
//                    ", wristDeg, " + df3.format(m_wristPos_deg) +
//                    ", wristPosV, " + df3.format(m_wristServoPos) +
//                    ", bcnMode, " + m_isBeaconMode +
//                    ", wheelSpd, " + df3.format(m_intakeWheelSpeed) +
//                    ", wheelOSpd, " + df3.format(m_intakeOverrideWheelSpeed) +
//                    ", ejectDelay, " + m_ejectDelayCounter +
//                    ", initDelay, " + m_initDelayCounter +
                    ".");
        }

        if (true) {
            telemetry.addData("left",
                    df3.format(m_leftVal) +
                            " right " + df3.format(m_rightVal) +
                            " poseY " + df3.format(m_poseY_in));
            telemetry.update();
        }
    }
}
