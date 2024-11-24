package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.Constants;
import frc.robot.HardwareMap;

public class DeployerFSM {
	/* ======================== Constants ======================== */

	// FSM state definitions
	public enum DeployerFSMState {
		DEPLOY,
		RETRACT
	}

	/* ======================== Private variables ======================== */
	private DeployerFSMState currentState;
	private SparkPIDController pidController;
	private RelativeEncoder encoder;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax neoMotor;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DeployerFSM() {
		// Perform neo init
		neoMotor = new CANSparkMax(
			HardwareMap.PIVOT_MOTOR_ID,
			CANSparkMax.MotorType.kBrushless);

		neoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		neoMotor.getEncoder().setPosition(0);

		pidController = neoMotor.getPIDController();
		encoder = neoMotor.getEncoder();

		pidController.setP(Constants.P);
		pidController.setI(Constants.I);
		pidController.setD(Constants.D);
		pidController.setIZone(Constants.IZ);
		pidController.setFF(Constants.FF);
		pidController.setOutputRange(Constants.MIN_OUT, Constants.MAX_OUT);

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public DeployerFSMState getCurrentState() {
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
		currentState = DeployerFSMState.RETRACT;
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
			case DEPLOY:
				handleDeployState(input);
				break;
			case RETRACT:
				handleRetractState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		currentState = nextState(input);

		SmartDashboard.putNumber("POS", encoder.getPosition());
		SmartDashboard.putNumber("VELO", encoder.getVelocity());
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
	private DeployerFSMState nextState(TeleopInput input) {
		switch (currentState) {
			case RETRACT:
				if (input.isDeployButtonPressed()) {
					return DeployerFSMState.DEPLOY;
				} else {
					return DeployerFSMState.RETRACT;
				}
			case DEPLOY:
				if (input.isDeployButtonPressed()) {
					return DeployerFSMState.DEPLOY;
				} else {
					return DeployerFSMState.RETRACT;
				}
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in DEPLOY.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleDeployState(TeleopInput input) {
		pidController.setReference(Constants.DEPLOYED_POS, ControlType.kPosition);
	}

	/**
	 * Handle behavior in RETRACT.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleRetractState(TeleopInput input) {
		pidController.setReference(Constants.HOME_POS, ControlType.kPosition);
	}
}
