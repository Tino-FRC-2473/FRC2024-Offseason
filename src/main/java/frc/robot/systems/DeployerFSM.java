package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

// Robot Imports
import frc.robot.Constants;
import frc.robot.HardwareMap;
import frc.robot.TeleopInput;

public class DeployerFSM {
	/* ======================== Constants ======================== */

	// FSM state definitions
	public enum DeployerFSMState {
		DEPLOY,
		RETRACT,
		IDLE
	}

	private final MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0);

	/* ======================== Private variables ======================== */
	private DeployerFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	// private CANSparkMax neoMotor;
	private TalonFX krakenMotor;

	//PID Controller for NEO motor
	// PIDController neoPID;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DeployerFSM() {
		// Perform neo init
		// neoMotor = new CANSparkMax(
		// 	NEO_MOTOR_ID,
		// 	CANSparkMax.MotorType.kBrushless);

		// neoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		// neoMotor.getEncoder().setPosition(0);

		// neoPID = new PIDController(NEO_P_CONSTANT, NEO_I_CONSTANT, NEO_D_CONSTANT);
		// neoPID.setTolerance(NEO_PID_POS_TOLERANCE, NEO_PID_VEL_TOLERANCE);
		// neoPID.reset();

		//perform kraken init
		krakenMotor = new TalonFX(HardwareMap.PIVOT_MOTOR_ID);
		krakenMotor.setNeutralMode(NeutralModeValue.Brake);

		var talonFXConfigs = new TalonFXConfiguration();

		// set slot 0 gains
		var slot0Configs = talonFXConfigs.Slot0;
		slot0Configs.kG = Constants.MM_CONSTANT_G; // Voltae output to overcome gravity
		slot0Configs.kS = Constants.MM_CONSTANT_S; // Voltage output to overcome static friction
		slot0Configs.kV = Constants.MM_CONSTANT_V; // Voltage for velocity target of 1 rps
		slot0Configs.kA = Constants.MM_CONSTANT_A; // Voltage for acceleration of 1 rps/s
		slot0Configs.kP = Constants.MM_CONSTANT_P; // Account for position error of 1 rotations
		slot0Configs.kI = Constants.MM_CONSTANT_I; // output for integrated error
		slot0Configs.kD = Constants.MM_CONSTANT_D; // Account for velocity error of 1 rps

		// set Motion Magic settings
		var motionMagicConfigs = talonFXConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = Constants.CONFIG_CONSTANT_CV; //Target velo
		motionMagicConfigs.MotionMagicAcceleration = Constants.CONFIG_CONSTANT_A; //Target accel
		motionMagicConfigs.MotionMagicJerk = Constants.CONFIG_CONSTANT_J; // Target jerk

		krakenMotor.getConfigurator().apply(talonFXConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
			Constants.UPDATE_FREQUENCY_HZ,
			krakenMotor.getPosition(),
			krakenMotor.getVelocity(),
			krakenMotor.getAcceleration(),
			krakenMotor.getMotorVoltage());

		krakenMotor.optimizeBusUtilization();
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

		SmartDashboard.putNumber("Postion", krakenMotor.getPosition().getValueAsDouble());
		SmartDashboard.putNumber("Velo", krakenMotor.getVelocity().getValueAsDouble());
		SmartDashboard.putNumber("Accel", krakenMotor.getAcceleration().getValueAsDouble());
		SmartDashboard.putNumber("Voltage", krakenMotor.getMotorVoltage().getValueAsDouble());
		SmartDashboard.putString("CURRENT STATE", currentState.toString());
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

				if (input == null) {
					return DeployerFSMState.RETRACT;
				}

				if (input.isIntakeButtonPressed()) {
					return DeployerFSMState.DEPLOY;
				} else {
					return DeployerFSMState.RETRACT;
				}

			case DEPLOY:
				if (input.isIntakeButtonPressed()) {
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
		krakenMotor.setControl(mmVoltage.withPosition(Constants.DEPLOYED_POSITION));
	}

	/**
	 * Handle behavior in RETRACT.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleRetractState(TeleopInput input) {
		krakenMotor.setControl(mmVoltage.withPosition(Constants.HOME_POSITION));
	}
}
