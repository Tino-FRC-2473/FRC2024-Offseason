package frc.robot.systems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
// WPILib Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class DeployerFSM {
	/* ======================== Constants ======================== */

	// FSM state definitions
	public enum DeployerFSMState {
		IDLE,
        DEPLOY,
        RETRACT
	}

    final int NEO_MOTOR_ID = -1;
    final int KRAKEN_MOTOR_ID = -1;

    final double NEO_P_CONSTANT = 0;
    final double NEO_I_CONSTANT = 0;
    final double NEO_D_CONSTANT = 0;

    final double PID_POS_TOLERANCE = 5;
    final double PID_VEL_TOLERANCE = 1;

    final double NEO_SETPOINT = 500;
    final double KRAKEN_SETPOINT = 2; // rotations

	/* ======================== Private variables ======================== */
	private DeployerFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax neoMotor;
	private TalonFX krakenMotor;

    //PID Controller for NEO motor
    PIDController neoPID;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DeployerFSM() {
		// Perform neo init
		neoMotor = new CANSparkMax(
			NEO_MOTOR_ID,
			CANSparkMax.MotorType.kBrushless);

        neoMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        neoMotor.getEncoder().setPosition(0);

        neoPID = new PIDController(NEO_P_CONSTANT, NEO_I_CONSTANT, NEO_D_CONSTANT);
        neoPID.setTolerance(PID_POS_TOLERANCE, PID_VEL_TOLERANCE);
        neoPID.reset();

        //perform kraken init
        krakenMotor = new TalonFX(KRAKEN_MOTOR_ID);
		krakenMotor.setNeutralMode(NeutralModeValue.Brake);

        // below ripped from ctre documentation:
        // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html
        var talonFXConfigs = new TalonFXConfiguration();

		krakenMotor.setControl(new VoltageOut(0.0)); //increment by 0.01 until motor starts moving the deployer to find kS

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kG = 0; // Add 0.00 V output to overcome gravity
        slot0Configs.kS = 0.25; // Add kS V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in kV V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires kA V output
        slot0Configs.kP = 4.8; // A position error of 1 rotations results in kP V output
        slot0Configs.kI = 0; // output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in kD V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 30; // Target cruise velocity of 30 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        krakenMotor.getConfigurator().apply(talonFXConfigs);

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
		currentState = DeployerFSMState.IDLE;
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
			case IDLE:
				handleIdleState(input);
				break;
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
			case IDLE:
				if (input.isIntakeButtonPressed()) {
					return DeployerFSMState.DEPLOY;
				} else {
					return DeployerFSMState.IDLE;
				}
			case RETRACT:
				if (input.isIntakeButtonPressed()) {
					return DeployerFSMState.DEPLOY;
				} else if (Math.abs(krakenMotor.getPosition().getValueAsDouble()) <= PID_POS_TOLERANCE) {
					return DeployerFSMState.IDLE;
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
	 * Handle behavior in IDLE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		krakenMotor.set(0);

		// neoMotor.set(0);
	}

    /**
	 * Handle behavior in DEPLOY.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleDeployState(TeleopInput input) {
		krakenMotor.setControl(new MotionMagicVoltage(0).withPosition(KRAKEN_SETPOINT));

        // neoMotor.set(neoPID.calculate(neoMotor.getEncoder().getPosition(), NEO_SETPOINT));
	}

    /**
	 * Handle behavior in RETRACT.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleRetractState(TeleopInput input) {
		krakenMotor.setControl(new MotionMagicVoltage(0).withPosition(0));

        // neoMotor.set(neoPID.calculate(neoMotor.getEncoder().getPosition(), 0));
	}
}
