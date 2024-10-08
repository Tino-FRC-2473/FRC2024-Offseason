package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class ClimberMechFSM {
	/* ======================== Constants ======================== */

	// FSM state definitions
	public enum ClimberMechFSMState {
		START_STATE,
		RAISE_HOOK_MANUAL,
		LOWER_HOOK_MANUAL,
		IDLE
	}

	private static final float MOTOR_POWER_UP = -0.5f;
	private static final float MOTOR_POWER_DOWN = 0.5f;

	private static final float RIGHT_RAISED_POSITION = -70f;

	private static final float LEFT_RAISED_POSITION = 70f;

	private static final float[] THRESHOLDS = new float[] {0.9f, 0.8f, 0.7f};
	private static final float[] MODIFIERS = new float[] {0.3f, 0.5f, 0.7f};

	/* ======================== Private variables ======================== */
	private ClimberMechFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax rightMotor;
	private CANSparkMax leftMotor;

	private SparkLimitSwitch leftBottomSwitch;
	private SparkLimitSwitch rightBottomSwitch;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ClimberMechFSM() {
		// Perform hardware init
		rightMotor = new CANSparkMax(
			HardwareMap.LEFT_CLIMBER_CAN_ID,
			CANSparkMax.MotorType.kBrushless);

		rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rightMotor.getEncoder().setPosition(0);

		leftMotor = new CANSparkMax(
			HardwareMap.RIGHT_CLIMBER_CAN_ID,
			CANSparkMax.MotorType.kBrushless);

		leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		leftMotor.getEncoder().setPosition(0);

		leftBottomSwitch = leftMotor.getReverseLimitSwitch(Type.kNormallyClosed);
		rightBottomSwitch = rightMotor.getForwardLimitSwitch(Type.kNormallyClosed);

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
		if (input == null) {
			return;
		}
		switch (currentState) {
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
		SmartDashboard.putString("Climber State", currentState.toString());
		SmartDashboard.putNumber("left encoder position", leftMotor.getEncoder().getPosition());
		SmartDashboard.putNumber("right encoder position", rightMotor.getEncoder().getPosition());

		currentState = nextState(input);
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
		ClimberMechFSMState next;

		switch (currentState) {
			case RAISE_HOOK_MANUAL:
				if (input.isManualRaiseButtonPressed() && !input.isManualLowerButtonPressed()) {
					next = ClimberMechFSMState.RAISE_HOOK_MANUAL;
				} else {
					next = ClimberMechFSMState.IDLE;
				}
				break;

			case LOWER_HOOK_MANUAL:
				if ((!input.isManualRaiseButtonPressed() && input.isManualLowerButtonPressed())) {
					next = ClimberMechFSMState.LOWER_HOOK_MANUAL;
				} else {
					next = ClimberMechFSMState.IDLE;
				}
				break;

			case IDLE:
				if (input.isManualRaiseButtonPressed() && !input.isManualLowerButtonPressed()) {
					next = ClimberMechFSMState.RAISE_HOOK_MANUAL;
				} else if (
					(input.isManualLowerButtonPressed() && !input.isManualRaiseButtonPressed())) {
					next = ClimberMechFSMState.LOWER_HOOK_MANUAL;
				} else {
					next = ClimberMechFSMState.IDLE;
				}
				break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		return next;
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
		rightMotor.set(calculatePower(true, true));
		leftMotor.set(calculatePower(true, false));
	}

	/**
	 * Handle behavior in LOWER_HOOK_MANUAL state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleLowerHookManualState(TeleopInput input) {
		//relying on sparkmax firmware, software redundancy exists
		if (rightBottomSwitch.isPressed()) {
			rightMotor.getEncoder().setPosition(0);
			rightMotor.set(0);
		} else {
			rightMotor.set(calculatePower(false, true));
		}
		if (leftBottomSwitch.isPressed()) {
			leftMotor.getEncoder().setPosition(0);
			leftMotor.set(0);
		} else {
			leftMotor.set(calculatePower(false, false));
		}
	}

	/**
	 * modifies the power going up based on a step function.
	 * @param goingUp whether or not the motor is going up
	 * @param right whether it gets the encoder information from the right or left motor
	 * @return the power to set to the motors based on the function modificatin
	 */
	private double calculatePower(boolean goingUp, boolean right) {
		double value = goingUp ? MOTOR_POWER_UP : MOTOR_POWER_DOWN;
		double raisedPosition;
		double currentPosition;
		if (right) {
			raisedPosition = goingUp ? RIGHT_RAISED_POSITION : RIGHT_RAISED_POSITION;
			currentPosition = rightMotor.getEncoder().getPosition();
		} else {
			value *= -1;
			raisedPosition = goingUp ? LEFT_RAISED_POSITION : LEFT_RAISED_POSITION;
			currentPosition = leftMotor.getEncoder().getPosition();
		}
		currentPosition = Math.abs(currentPosition);
		raisedPosition = Math.abs(raisedPosition);
		if (currentPosition >= THRESHOLDS[0] * raisedPosition) {
			return MODIFIERS[0] * value;
		} else if (currentPosition >= THRESHOLDS[1] * raisedPosition) {
			return MODIFIERS[1] * value;
		} else if (currentPosition >= THRESHOLDS[2] * raisedPosition) {
			return MODIFIERS[2] * value;
		}
		return value;
	}
}
