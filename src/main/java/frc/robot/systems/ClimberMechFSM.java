package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class ClimberMechFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum ClimberMechFSMState {
		IDLE_STOP,
		CLIMBING,
		HOOKS_UP
	}

	private static final float SYNCH_MOTOR_POWER = 0.5f; //0.25
	private static final float UP_MOTOR_POWER = 0.25f;
	private static final float HOOKS_UP_ENCODER = 11.00f;
	private static final float CHAIN_ENCODER = 35.57f;


	/* ======================== Private variables ======================== */
	private ClimberMechFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax motorLeft;
    private CANSparkMax motorRight;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ClimberMechFSM() {
		// Perform hardware init
		motorLeft = new CANSparkMax(HardwareMap.LEFT_CLIMBER_CAN_ID,
						CANSparkMax.MotorType.kBrushless);
		motorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
		motorLeft.getEncoder().setPosition(0);

        motorRight = new CANSparkMax(HardwareMap.RIGHT_CLIMBER_CAN_ID,
						CANSparkMax.MotorType.kBrushless);
		motorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
		motorRight.getEncoder().setPosition(0);

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
		SmartDashboard.putString("Left Climber State", currentState.toString());

		currentState = nextState(input);
		// SmartDashboard.putNumber("left output", motor.getAppliedOutput());
		// SmartDashboard.putNumber("left motor applied", motor.get());
		SmartDashboard.putNumber("left encoder position", motorLeft.getEncoder().getPosition());
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
        double currentEncoderLeft = motorLeft.getEncoder().getPosition();
        double currentEncoderRight = motorRight.getEncoder().getPosition();

		switch (currentState) {
			case IDLE_STOP:
				if (input.synchClimberTrigger() && 
                    (HOOKS_UP_ENCODER >= currentEncoderLeft && currentEncoderLeft > CHAIN_ENCODER) &&
                    (HOOKS_UP_ENCODER >= currentEncoderRight && currentEncoderRight > CHAIN_ENCODER)) {
					return ClimberMechFSMState.CLIMBING;
				} else if (!input.isHooksUpButtonPressed() || 
                    (currentEncoderLeft <= HOOKS_UP_ENCODER && currentEncoderRight <= HOOKS_UP_ENCODER)) {
					return ClimberMechFSMState.HOOKS_UP;
				} else {
					return ClimberMechFSMState.IDLE_STOP;
				}
			case CLIMBING:
				if (input.synchClimberTrigger() && 
                (currentEncoderLeft > CHAIN_ENCODER && currentEncoderRight > CHAIN_ENCODER)) {
					return ClimberMechFSMState.CLIMBING;
				} else if (!input.synchClimberTrigger() || 
					(currentEncoderLeft <= CHAIN_ENCODER && currentEncoderRight <= CHAIN_ENCODER)) {
					return ClimberMechFSMState.IDLE_STOP;
				} else {
					return ClimberMechFSMState.CLIMBING;
				}
			case HOOKS_UP:
				if (input.isHooksUpButtonPressed() && 
					(currentEncoderLeft > HOOKS_UP_ENCODER && currentEncoderRight > HOOKS_UP_ENCODER)) {
					return ClimberMechFSMState.HOOKS_UP;
				} else if (!input.isHooksUpButtonPressed() || 
				(currentEncoderLeft <= HOOKS_UP_ENCODER && currentEncoderRight <= HOOKS_UP_ENCODER)) {
					return ClimberMechFSMState.IDLE_STOP;
				} else {
					return ClimberMechFSMState.HOOKS_UP;
				}
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
		motorLeft.set(0);
        motorRight.set(0);
	}
	/**
	 * Handle behavior in CLIMBING state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleClimbingState(TeleopInput input) {
		if (motorLeft.getEncoder().getPosition() <= CHAIN_ENCODER) {
			motorLeft.set(SYNCH_MOTOR_POWER);
		} else {
			motorLeft.set(0);
		}

        if (motorRight.getEncoder().getPosition() <= CHAIN_ENCODER) {
			motorRight.set(SYNCH_MOTOR_POWER);
		} else {
			motorRight.set(0);
		}
	}
	/**
	 * Handle behavior in HOOKS_UP state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleHooksUpState(TeleopInput input) {
		if (motorLeft.getEncoder().getPosition() <= HOOKS_UP_ENCODER) {
			motorLeft.set(UP_MOTOR_POWER);
		} else {
			motorLeft.set(0);
		}

        if (motorRight.getEncoder().getPosition() <= HOOKS_UP_ENCODER) {
			motorRight.set(UP_MOTOR_POWER);
		} else {
			motorRight.set(0);
		}
	}
}
