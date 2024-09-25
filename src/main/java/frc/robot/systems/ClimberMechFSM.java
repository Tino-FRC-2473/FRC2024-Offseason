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

	private static final float MOTOR_POWER_UP = 0.1f;
	private static final float MOTOR_POWER_DOWN = -0.1f;

	private static final float RIGHT_RAISED_POSITION = 100f;
	private static final float RIGHT_FINAL_POSITION = 0f;

	private static final float LEFT_RAISED_POSITION = -RIGHT_RAISED_POSITION;
	private static final float LEFT_FINAL_POSITION = -RIGHT_FINAL_POSITION;

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
			HardwareMap.CAN_ID_SPARK_LEFT_CLIMBER_MOTOR,
			CANSparkMax.MotorType.kBrushless);

		rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rightMotor.getEncoder().setPosition(0);

		leftMotor = new CANSparkMax(
			HardwareMap.CAN_ID_SPARK_LEFT_CLIMBER_MOTOR,
			CANSparkMax.MotorType.kBrushless);

		leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		leftMotor.getEncoder().setPosition(0);

		leftBottomSwitch = leftMotor.getReverseLimitSwitch(Type.kNormallyClosed);
		rightBottomSwitch = rightMotor.getReverseLimitSwitch(Type.kNormallyClosed);

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
				if ((!input.isManualRaiseButtonPressed() && input.isManualLowerButtonPressed())
					&& !(leftBottomSwitch.isPressed() && rightBottomSwitch.isPressed())) {
					next = ClimberMechFSMState.LOWER_HOOK_MANUAL;
				} else {
					next = ClimberMechFSMState.IDLE;
				}
				break;

			case IDLE:
				if (input.isManualRaiseButtonPressed() && !input.isManualLowerButtonPressed()) {
					next = ClimberMechFSMState.RAISE_HOOK_MANUAL;
				} else if (
					(input.isManualLowerButtonPressed() && !input.isManualRaiseButtonPressed())
						&& !(leftBottomSwitch.isPressed() && rightBottomSwitch.isPressed())) {
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
		if (rightMotor.getEncoder().getPosition() < RIGHT_RAISED_POSITION) {
			rightMotor.set(MOTOR_POWER_UP);
		} else {
			rightMotor.set(0);
		}

		if (leftMotor.getEncoder().getPosition() > LEFT_RAISED_POSITION) {
			leftMotor.set(-MOTOR_POWER_UP);
		} else {
			leftMotor.set(0);
		}
	}

	/**
	 * Handle behavior in LOWER_HOOK_MANUAL state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleLowerHookManualState(TeleopInput input) {
		if (rightMotor.getEncoder().getPosition() > RIGHT_FINAL_POSITION
			&& !rightBottomSwitch.isPressed()) {
			rightMotor.set(MOTOR_POWER_DOWN);
		} else {
			rightMotor.set(0);
		}

		if (leftMotor.getEncoder().getPosition() < LEFT_FINAL_POSITION
			&& !leftBottomSwitch.isPressed()) {
			leftMotor.set(-MOTOR_POWER_DOWN);
		} else {
			leftMotor.set(0);
		}
	}
}
