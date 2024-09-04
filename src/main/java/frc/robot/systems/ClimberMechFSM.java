package frc.robot.systems;

import edu.wpi.first.wpilibj.DigitalInput;
// WPILib Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class ClimberMechFSM {
	/* ======================== Constants ======================== */

	// FSM state definitions
	public enum ClimberMechFSMState {
		START_STATE,
        LOWER_HOOK_AUTO,
        RAISE_HOOK_AUTO,
        RAISE_HOOK_MANUAL,
        LOWER_HOOK_MANUAL,
        IDLE

	}

	private static final float SYNCH_MOTOR_POWER = 0.5f; //0.25
	private static final float MOTOR_POWER_UP = 0.25f;
    private static final float MOTOR_POWER_DOWN = -0.25f;

    private static final float RIGHT_DEFAULT_POSITION = 0f;
	private static final float RIGHT_RAISED_POSITION = 0f;
    private static final float RIGHT_FINAL_POSITION = 0f;

    private static final float LEFT_DEFAULT_POSITION = -RIGHT_DEFAULT_POSITION;
	private static final float LEFT_RAISED_POSITION = -RIGHT_RAISED_POSITION;
    private static final float LEFT_FINAL_POSITION = -RIGHT_FINAL_POSITION;

	/* ======================== Private variables ======================== */
	private ClimberMechFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax rightMotor;
    private CANSparkMax leftMotor;
    
    private DigitalInput rightMaxLimitSwitch;
    private DigitalInput rightMinLimitSwitch;
    private DigitalInput leftMaxLimitSwitch;
    private DigitalInput leftMinLimitSwitch;


	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ClimberMechFSM() {
		// Perform hardware init
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_LEFT_CLIMBER_MOTOR,CANSparkMax.MotorType.kBrushless);
		rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	    rightMotor.getEncoder().setPosition(0);

        leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_LEFT_CLIMBER_MOTOR,CANSparkMax.MotorType.kBrushless);
		leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	    leftMotor.getEncoder().setPosition(0);

        rightMaxLimitSwitch = new DigitalInput(HardwareMap.CLIMBER_RIGHT_MAX_LIMIT_SWITCH_CHANNEL);
        rightMinLimitSwitch = new DigitalInput(HardwareMap.CLIMBER_RIGHT_MIN_LIMIT_SWITCH_CHANNEL);
        leftMaxLimitSwitch = new DigitalInput(HardwareMap.CLIMBER_LEFT_MAX_LIMIT_SWITCH_CHANNEL);
        leftMinLimitSwitch = new DigitalInput(HardwareMap.CLIMBER_LEFT_MIN_LIMIT_SWITCH_CHANNEL);

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public ClimberMechFSMState getCurrentState() {
		return currentState;
	}
	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = ClimberMechFSMState.IDLE;
		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
        if (input == null) return;
		switch (currentState) {
			case LOWER_HOOK_AUTO:
                handleLowerHookAutoState(input);
				break;
			case RAISE_HOOK_AUTO:
                handleRaiseHookAutoState(input);
				break;
			case RAISE_HOOK_MANUAL:
                handleRaiseHookManualState(input);
				break;
            case LOWER_HOOK_MANUAL:
                handleLowerHookManualState(input);
                break;
            case IDLE:
                handleIdleState(input);
                break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		SmartDashboard.putString("Left Climber State", currentState.toString());
		currentState = nextState(input);
		SmartDashboard.putNumber("left encoder position", leftMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("right encoder position", rightMotor.getEncoder().getPosition());
	}


	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private ClimberMechFSMState nextState(TeleopInput input) {
		switch (currentState) {
            case RAISE_HOOK_AUTO:
                if (rightMotor.getEncoder().getPosition() > RIGHT_RAISED_POSITION && leftMotor.getEncoder().getPosition() < LEFT_RAISED_POSITION) return ClimberMechFSMState.IDLE;
                else return ClimberMechFSMState.RAISE_HOOK_AUTO;
			case LOWER_HOOK_AUTO:
                if (rightMotor.getEncoder().getPosition() < RIGHT_FINAL_POSITION && leftMotor.getEncoder().getPosition() > LEFT_FINAL_POSITION) return ClimberMechFSMState.IDLE;
                else return ClimberMechFSMState.RAISE_HOOK_AUTO;
			case RAISE_HOOK_MANUAL:
                if (!input.isManualRaiseButtonPressed() || rightMotor.getEncoder().getPosition() > RIGHT_RAISED_POSITION && leftMotor.getEncoder().getPosition() < LEFT_RAISED_POSITION || rightMaxLimitSwitch.get() && leftMaxLimitSwitch.get()) return ClimberMechFSMState.IDLE;
                else return ClimberMechFSMState.RAISE_HOOK_MANUAL;
            case LOWER_HOOK_MANUAL:
                if (!input.isManualLowerButtonPressed() || rightMotor.getEncoder().getPosition() < RIGHT_DEFAULT_POSITION && leftMotor.getEncoder().getPosition() > LEFT_DEFAULT_POSITION || rightMinLimitSwitch.get() && leftMinLimitSwitch.get()) return ClimberMechFSMState.IDLE;
                else return ClimberMechFSMState.LOWER_HOOK_MANUAL;
            case IDLE:
                if () return ClimberMechFSMState.LOWER_HOOK_AUTO;
                if () return ClimberMechFSMState.RAISE_HOOK_AUTO;
                if (input.isManualRaiseButtonPressed()) return ClimberMechFSMState.RAISE_HOOK_MANUAL;
                if (input.isManualLowerButtonPressed()) return ClimberMechFSMState.LOWER_HOOK_MANUAL;
                else return ClimberMechFSMState.IDLE;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		leftMotor.set(0);
        rightMotor.set(0);
	}
	/**
	 * Handle behavior in RAISE_HOOK_MANUAL state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleRaiseHookManualState(TeleopInput input) {
		rightMotor.set(MOTOR_POWER_UP);
        leftMotor.set(MOTOR_POWER_DOWN);
	}
    /**
	 * Handle behavior in LOWER_HOOK_MANUAL state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleLowerHookManualState(TeleopInput input) {
		rightMotor.set(MOTOR_POWER_DOWN);
        leftMotor.set(MOTOR_POWER_UP);
	}
    /**
	 * Handle behavior in RAISE_HOOK_AUTO state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleRaiseHookAutoState(TeleopInput input) {
		rightMotor.set(MOTOR_POWER_UP);
        leftMotor.set(MOTOR_POWER_DOWN);

	}
    /**
	 * Handle behavior in LOWER_HOOK_AUTO state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleLowerHookAutoState(TeleopInput input) {
        rightMotor.set(MOTOR_POWER_DOWN);
        leftMotor.set(MOTOR_POWER_UP);
	}
}