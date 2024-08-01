package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class ClimberMechFSMRight {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum ClimberMechFSMState {
		IDLE_STOP,
		CLIMBING,
		HOOKS_UP
	}

	private static final float SYNCH_MOTOR_POWER = -0.5f; //-0.25
	private static final float UP_MOTOR_POWER = -0.25f; //-0.25

	private static final float PEAK_ENCODER_POSITION = -0.57f;
	private static final float CLIMB_ENCODER_POSITION = -1.73f;

	/* ======================== Private variables ======================== */
	private ClimberMechFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax motor;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ClimberMechFSMRight() {
		// Perform hardware init
		motor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_RIGHT_CLIMBER_MOTOR,
						CANSparkMax.MotorType.kBrushless);
		motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		motor.getEncoder().setPosition(0);

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
		currentState = ClimberMechFSMState.IDLE_STOP;
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
			case IDLE_STOP:
				handleIdleState(input);
				break;
			case CLIMBING:
				handleClimbingState(input);
				break;
			case HOOKS_UP:
				handleHooksUpState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		SmartDashboard.putString("Right Climber State", currentState.toString());

		currentState = nextState(input);
		// SmartDashboard.putNumber("right output", motor.getAppliedOutput());
		// SmartDashboard.putNumber("right motor applied", motor.get());
		SmartDashboard.putNumber("right encoder position", motor.getEncoder().getPosition());

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
			case IDLE_STOP:
				if (input.synchClimberTrigger() && !input.isHooksUpButtonPressed()) {
					return ClimberMechFSMState.CLIMBING;
				} else if (!input.synchClimberTrigger() && input.isHooksUpButtonPressed()) {
					return ClimberMechFSMState.HOOKS_UP;
				} else {
					return ClimberMechFSMState.IDLE_STOP;
				}
			case CLIMBING:
				if (input.synchClimberTrigger() && !input.isHooksUpButtonPressed()) {
					return ClimberMechFSMState.CLIMBING;
				} else {
					return ClimberMechFSMState.IDLE_STOP;
				}
			case HOOKS_UP:
				if (!input.synchClimberTrigger() && input.isHooksUpButtonPressed()) {
					return ClimberMechFSMState.HOOKS_UP;
				} else {
					return ClimberMechFSMState.IDLE_STOP;
				}
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in IDLE_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		motor.set(0);
	}
	/**
	 * Handle behavior in CLIMBING state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleClimbingState(TeleopInput input) {
		if (motor.getEncoder().getPosition() >= CLIMB_ENCODER_POSITION) {
			motor.set(SYNCH_MOTOR_POWER);
		} else {
			motor.set(0);
		}
	}
	/**
	 * Handle behavior in HOOKS_UP state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleHooksUpState(TeleopInput input) {
		if (motor.getEncoder().getPosition() >= PEAK_ENCODER_POSITION) {
			motor.set(UP_MOTOR_POWER);
		} else {
			motor.set(0);
		}
	}
}