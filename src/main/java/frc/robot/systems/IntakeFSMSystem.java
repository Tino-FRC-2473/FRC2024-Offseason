package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.LED;
import frc.robot.MechConstants;

public class IntakeFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum IntakeFSMState {
		MOVE_TO_HOME,
		MOVE_TO_GROUND,
		INTAKING,
		OUTTAKING,
		FEED_SHOOTER,
	}

	/* ======================== Private variables ======================== */
	private IntakeFSMState currentState;
	private boolean hasNote = false;
	private int noteColorFrames = 0;
	private Timer timer;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax frontIndexMotor;
	private CANSparkMax backIndexMotor;
	private TalonFX topIntakeMotor;
	private TalonFX bottomIntakeMotor;
	private TalonFX pivotMotor;

	private Encoder throughBore;
	private final ColorSensorV3 colorSensor;
	private LED led;

	/* ======================== Constructor ======================== */
	/**
	 * Create IntakeFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public IntakeFSMSystem() {
		// Perform hardware init
		frontIndexMotor = new CANSparkMax(HardwareMap.FRONT_INDEXER_MOTOR_ID,
										CANSparkMax.MotorType.kBrushless);
		frontIndexMotor.setIdleMode(IdleMode.kBrake);

		backIndexMotor = new CANSparkMax(HardwareMap.BACK_INDEXER_MOTOR_ID,
										CANSparkMax.MotorType.kBrushless);
		backIndexMotor.setIdleMode(IdleMode.kBrake);

		pivotMotor = new TalonFX(HardwareMap.PIVOT_MOTOR_ID);
		pivotMotor.setNeutralMode(NeutralModeValue.Brake);

		topIntakeMotor = new TalonFX(HardwareMap.TOP_INTAKE_MOTOR_ID);
		topIntakeMotor.setNeutralMode(NeutralModeValue.Brake);

		bottomIntakeMotor = new TalonFX(HardwareMap.BOTTOM_INTAKE_MOTOR_ID);
		bottomIntakeMotor.setNeutralMode(NeutralModeValue.Brake);

		throughBore = new Encoder(0, 1);
		throughBore.reset();

		colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

		timer = new Timer();

		led = new LED();

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public IntakeFSMState getCurrentState() {
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
		led.greenLight(false);
		currentState = IntakeFSMState.MOVE_TO_HOME;
		hasNote = false;

		timer.stop();
		timer.reset();
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
		switch (currentState) {
			case MOVE_TO_HOME:
				handleMoveHomeState(input);
				break;

			case MOVE_TO_GROUND:
				handleMoveGroundState(input);
				break;

			case INTAKING:
				handleIntakingState(input);
				break;

			case OUTTAKING:
				handleOuttakingState(input);
				break;

			case FEED_SHOOTER:
				handleFeedShooterState(input);
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
	private IntakeFSMState nextState(TeleopInput input) {
		switch (currentState) {
			case MOVE_TO_HOME:
				if (input == null) {
					return IntakeFSMState.MOVE_TO_HOME;
				}

				if ((input.isIntakeButtonPressed() || input.isOuttakeButtonPressed())
					&& !input.isShootButtonPressed()) {
					return IntakeFSMState.MOVE_TO_GROUND;
				}

				if (input.isShootButtonPressed() && !(input.isIntakeButtonPressed()
					|| input.isOuttakeButtonPressed())) {
					return IntakeFSMState.FEED_SHOOTER;
				}

			case FEED_SHOOTER:
				if (input.isShootButtonPressed() && !(input.isIntakeButtonPressed()
					|| input.isOuttakeButtonPressed())) {
					return IntakeFSMState.FEED_SHOOTER;
				}

				if (!input.isShootButtonPressed() || input.isIntakeButtonPressed()
					|| input.isOuttakeButtonPressed()) {
					return IntakeFSMState.MOVE_TO_HOME;
				}

			case MOVE_TO_GROUND:
				if (inRange(throughBore.getDistance(), MechConstants.GROUND_ENCODER_ROTATIONS)) {
					if (input.isIntakeButtonPressed() && !input.isOuttakeButtonPressed()
						&& !hasNote && !input.isShootButtonPressed()) {
						return IntakeFSMState.INTAKING;
					} else if (input.isOuttakeButtonPressed() && !input.isIntakeButtonPressed()
								&& input.isShootButtonPressed()) {
						return IntakeFSMState.OUTTAKING;
					}
				} else if (!inRange(throughBore.getDistance(),
					MechConstants.GROUND_ENCODER_ROTATIONS)) {
					if ((input.isIntakeButtonPressed() || input.isOuttakeButtonPressed())
						&& !input.isShootButtonPressed()) {
						return IntakeFSMState.MOVE_TO_GROUND;
					}
				}

				if ((!input.isIntakeButtonPressed() && !input.isOuttakeButtonPressed())
					|| input.isShootButtonPressed()) {
					return IntakeFSMState.MOVE_TO_HOME;
				}

			case INTAKING:
				if (inRange(throughBore.getDistance(), MechConstants.GROUND_ENCODER_ROTATIONS)
					&& input.isIntakeButtonPressed() && !hasNote
					&& !input.isOuttakeButtonPressed() && !input.isShootButtonPressed()) {
					return IntakeFSMState.INTAKING;
				}

				if (input.isShootButtonPressed() || hasNote) {
					return IntakeFSMState.MOVE_TO_HOME;
				}

				if ((!input.isIntakeButtonPressed() && !hasNote)
					|| input.isOuttakeButtonPressed()) {
					return IntakeFSMState.MOVE_TO_GROUND;
				}

			case OUTTAKING:
				if (inRange(throughBore.getDistance(), MechConstants.GROUND_ENCODER_ROTATIONS)
					&& input.isOuttakeButtonPressed() && !input.isIntakeButtonPressed()
					&& !input.isShootButtonPressed()) {
					return IntakeFSMState.OUTTAKING;
				}

				if (input.isShootButtonPressed()) {
					return IntakeFSMState.MOVE_TO_HOME;
				}

				if (!input.isOuttakeButtonPressed() || input.isIntakeButtonPressed()) {
					return IntakeFSMState.MOVE_TO_GROUND;
				}

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

/* ------------------------ Command handlers ------------------------ */

	private boolean inRange(double a, double b) {
		return Math.abs(a - b) < MechConstants.INRANGE_VALUE; //EXPERIMENTAL
	}

	private double pid(double currentEncoderPID, double targetEncoder) {
		double correction = MechConstants.PID_CONSTANT_PIVOT_P
			* (targetEncoder - currentEncoderPID);
		return Math.min(MechConstants.MAX_TURN_SPEED,
			Math.max(MechConstants.MIN_TURN_SPEED, correction));
	}

	private double pidAuto(double currentEncoderPID, double targetEncoder) {
		double correction = MechConstants.PID_CONSTANT_PIVOT_P_AUTO
			* (targetEncoder - currentEncoderPID);
		return Math.min(MechConstants.MAX_TURN_SPEED_AUTO,
			Math.max(MechConstants.MIN_TURN_SPEED_AUTO, correction));
	}

	/**
	 * Checks if the intake is holding a note.
	 * @return if the intake is holding a note
	 */
	public boolean hasNote() {
		boolean isInRange = colorSensor.getProximity() >= MechConstants.PROXIMIIY_THRESHOLD;
		SmartDashboard.putBoolean("is close enough", isInRange);

		noteColorFrames = isInRange ? (noteColorFrames + 1) : 0;
		hasNote = noteColorFrames >= MechConstants.NOTE_FRAMES_MIN;

		return hasNote;
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in MOVE_TO_HOME.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleMoveHomeState(TeleopInput input) {
		if (hasNote) {
			led.blueLight();
		} else {
			led.rainbow();
		}

		pivotMotor.set(pid(throughBore.getDistance(), MechConstants.HOME_ENCODER_ROTATION));
		topIntakeMotor.set(0);
		bottomIntakeMotor.set(0);
		frontIndexMotor.set(0);
		backIndexMotor.set(0);
	}

	/**
	 * Handle behavior in MOVE_TO_GROUND.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleMoveGroundState(TeleopInput input) {
		led.orangeLight(false);

		pivotMotor.set(pid(throughBore.getDistance(), MechConstants.GROUND_ENCODER_ROTATIONS));
		topIntakeMotor.set(0);
		bottomIntakeMotor.set(0);
		frontIndexMotor.set(0);
		backIndexMotor.set(0);
	}

	/**
	 * Handle behavior in MOVE_TO_HOME.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIntakingState(TeleopInput input) {
		if (!hasNote) {
			led.orangeLight(false);
		} else {
			led.greenLight(true);
		}

		pivotMotor.set(pid(throughBore.getDistance(), MechConstants.GROUND_ENCODER_ROTATIONS));
		bottomIntakeMotor.set(MechConstants.INTAKE_POWER);
		topIntakeMotor.set(MechConstants.INTAKE_POWER);
		frontIndexMotor.set(MechConstants.INTAKE_POWER);
		backIndexMotor.set(0);
	}

	/**
	 * Handle behavior in MOVE_TO_HOME.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOuttakingState(TeleopInput input) {
		if (hasNote) {
			led.orangeLight(false);
		} else {
			led.redLight(false);
		}

		pivotMotor.set(pid(throughBore.getDistance(), MechConstants.GROUND_ENCODER_ROTATIONS));
		bottomIntakeMotor.set(MechConstants.OUTTAKE_POWER);
		topIntakeMotor.set(MechConstants.OUTTAKE_POWER);
		frontIndexMotor.set(MechConstants.OUTTAKE_POWER);
		backIndexMotor.set(MechConstants.OUTTAKE_POWER);
	}

	/**
	 * Handle behavior in MOVE_TO_HOME.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleFeedShooterState(TeleopInput input) {
		led.blueLight();

		pivotMotor.set(pid(throughBore.getDistance(), MechConstants.HOME_ENCODER_ROTATION));
		topIntakeMotor.set(0);
		bottomIntakeMotor.set(0);
		frontIndexMotor.set(MechConstants.FEED_SHOOTER_POWER);
		backIndexMotor.set(MechConstants.FEED_SHOOTER_POWER);
	}


	private boolean handleAutoShootPreloaded() {
		if (timer.get() == 0) {
			timer.start();
		}
		pivotMotor.set(pid(throughBore.getDistance(), MechConstants.HOME_ENCODER_ROTATION));
		if (timer.get() < MechConstants.AUTO_PRELOAD_REVVING_TIME) {
			topIntakeMotor.set(0);
			bottomIntakeMotor.set(0);
			frontIndexMotor.set(0);
			backIndexMotor.set(0);
			return false;
		} else if (timer.get() < MechConstants.AUTO_PRELOAD_SHOOTING_TIME) {
			topIntakeMotor.set(0);
			bottomIntakeMotor.set(0);
			frontIndexMotor.set(MechConstants.FEED_SHOOTER_POWER);
			backIndexMotor.set(MechConstants.FEED_SHOOTER_POWER);
			return false;
		} else {
			topIntakeMotor.set(0);
			bottomIntakeMotor.set(0);
			frontIndexMotor.set(0);
			backIndexMotor.set(0);
			timer.stop();
			timer.reset();
			return true;
		}
	}

	/**
	 * Performs action for auto Move to Ground.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoMoveGround() {
		led.orangeLight(false);
		pivotMotor.set(pidAuto(throughBore.getDistance(), MechConstants.GROUND_ENCODER_ROTATIONS));
		return inRange(throughBore.getDistance(), MechConstants.GROUND_ENCODER_ROTATIONS);
	}

	/**
	 * Performs action for auto STATE2.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoMoveHome() {
		if (hasNote) {
			led.rainbow();
		} else {
			led.orangeLight(false);
		}

		pivotMotor.set(pidAuto(throughBore.getDistance(), MechConstants.HOME_ENCODER_ROTATION));
		return inRange(throughBore.getDistance(), MechConstants.HOME_ENCODER_ROTATION);
	}

	/**
	 * Performs action for auto STATE3.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoFeedShooter() {
		if (timer.get() == 0) {
			timer.start();
		}
		pivotMotor.set(pid(throughBore.getDistance(), MechConstants.HOME_ENCODER_ROTATION));
		if (timer.get() > MechConstants.AUTO_SHOOTING_TIME) {
			topIntakeMotor.set(0);
			bottomIntakeMotor.set(0);
			frontIndexMotor.set(0);
			backIndexMotor.set(0);
			timer.stop();
			timer.reset();
			return true;
		} else {
			topIntakeMotor.set(0);
			bottomIntakeMotor.set(0);
			frontIndexMotor.set(MechConstants.FEED_SHOOTER_POWER);
			backIndexMotor.set(MechConstants.FEED_SHOOTER_POWER);
			return false;
		}
	}

	private boolean handleAutoIntake() {
		topIntakeMotor.set(MechConstants.AUTO_INTAKE_POWER);
		bottomIntakeMotor.set(MechConstants.AUTO_INTAKE_POWER);
		frontIndexMotor.set(MechConstants.AUTO_INTAKE_POWER);
		backIndexMotor.set(0);
		return hasNote();
	}
}
