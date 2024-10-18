package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

// Third party Hardware Imports
import com.revrobotics.ColorSensorV3;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.LED;
import frc.robot.Constants;

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

	private final MotionMagicVelocityVoltage mVoltage = new MotionMagicVelocityVoltage(0);

	/* ======================== Private variables ======================== */
	private IntakeFSMState currentState;
	private boolean hasNote = false;
	private int noteColorFrames = 0;
	private LED led;
	private Timer timer = new Timer();
	private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
	private Slot0Configs slot0Configs = talonFXConfigs.Slot0;
	private MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
	private StatusCode statusCode = StatusCode.StatusCodeNotInitialized;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private TalonFX indexerMotor;
	private TalonFX intakeMotor;
	private TalonFX pivotMotor;

	private Encoder throughBore;
	private final ColorSensorV3 colorSensor;

	/* ======================== Constructor ======================== */
	/**
	 * Create IntakeFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public IntakeFSMSystem() {
		// Perform hardware init

		indexerMotor = new TalonFX(HardwareMap.INDEXER_MOTOR_ID);
		indexerMotor.setNeutralMode(NeutralModeValue.Brake);

		pivotMotor = new TalonFX(HardwareMap.PIVOT_MOTOR_ID);
		pivotMotor.setNeutralMode(NeutralModeValue.Brake);

		intakeMotor = new TalonFX(HardwareMap.INTAKE_MOTOR_ID);
		intakeMotor.setNeutralMode(NeutralModeValue.Coast);

		throughBore = new Encoder(HardwareMap.ENCODER_CHANNEL_A, HardwareMap.ENCODER_CHANNEL_B);
		throughBore.reset();

		colorSensor = new ColorSensorV3(Port.kOnboard);
		led = new LED();

		slot0Configs.kS = Constants.MM_CONSTANT_S;
		slot0Configs.kV = Constants.MM_CONSTANT_V;
		slot0Configs.kA = Constants.MM_CONSTANT_A;

		slot0Configs.kP = Constants.MM_CONSTANT_P;
		slot0Configs.kI = Constants.MM_CONSTANT_I;
		slot0Configs.kD = 0;

		motionMagicConfigs.MotionMagicAcceleration = Constants.CONFIG_CONSTANT_A;
		motionMagicConfigs.MotionMagicJerk = Constants.CONFIG_CONSTANT_J;

		statusCode = indexerMotor.getConfigurator().apply(talonFXConfigs);
		statusCode = intakeMotor.getConfigurator().apply(talonFXConfigs);

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
		// led.greenLight(false);
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
		hasNote = hasNote();

		SmartDashboard.putString("CURRENT STATE", currentState.toString());
		SmartDashboard.putNumber("Back Indexer Velocity",
			indexerMotor.getVelocity().getValueAsDouble());

		SmartDashboard.putBoolean("Color Sensor in Range?", colorSensor.getProximity()
			>= Constants.PROXIMIIY_THRESHOLD);
		SmartDashboard.putNumber("Frames with Note in View", noteColorFrames);
		SmartDashboard.putNumber("PIVOT ENCODER VAL", throughBore.getDistance());

		SmartDashboard.putBoolean("HASNOTE", hasNote);

		SmartDashboard.putNumber("intake velocity", intakeMotor.getVelocity().getValueAsDouble());
		SmartDashboard.putNumber("indexer velocity", indexerMotor.getVelocity().getValueAsDouble());
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

				if (input.isIntakeButtonPressed() || input.isOuttakeButtonPressed()) {
					return IntakeFSMState.MOVE_TO_GROUND;
				} else {
					if (input.isShootButtonPressed()) {
						return IntakeFSMState.FEED_SHOOTER;
					} else {
						return IntakeFSMState.MOVE_TO_HOME;
					}
				}

			case FEED_SHOOTER:
				if (input.isShootButtonPressed() && !(input.isIntakeButtonPressed()
					|| input.isOuttakeButtonPressed())) {
					return IntakeFSMState.FEED_SHOOTER;
				} else {
					return IntakeFSMState.MOVE_TO_HOME;
				}

			case MOVE_TO_GROUND:
				if (approxEquals(throughBore.getDistance(), Constants.GROUND_ENCODER_COUNT)) {
					if (input.isIntakeButtonPressed() && !input.isOuttakeButtonPressed()
						&& !input.isShootButtonPressed()) {
						return IntakeFSMState.INTAKING;
					} else if (input.isOuttakeButtonPressed() && !input.isIntakeButtonPressed()
								&& !input.isShootButtonPressed()) {
						return IntakeFSMState.OUTTAKING;
					}
				}

				if ((input.isIntakeButtonPressed() || input.isOuttakeButtonPressed())
					&& !input.isShootButtonPressed()) {
					return IntakeFSMState.MOVE_TO_GROUND;
				} else {
					return IntakeFSMState.MOVE_TO_HOME;
				}

			case INTAKING:
				if (approxEquals(throughBore.getDistance(), Constants.GROUND_ENCODER_COUNT)
					&& input.isIntakeButtonPressed() && !input.isOuttakeButtonPressed()
					&& !input.isShootButtonPressed()) {
					return IntakeFSMState.INTAKING;
				}

				if (input.isShootButtonPressed()
					&& !(input.isOuttakeButtonPressed() || input.isIntakeButtonPressed())) {
					return IntakeFSMState.MOVE_TO_HOME;
				} else {
					return IntakeFSMState.MOVE_TO_GROUND;
				}

			case OUTTAKING:
				if (approxEquals(throughBore.getDistance(), Constants.GROUND_ENCODER_COUNT)
					&& input.isOuttakeButtonPressed() && !input.isIntakeButtonPressed()
					&& !input.isShootButtonPressed()) {
					return IntakeFSMState.OUTTAKING;
				}

				if (input.isShootButtonPressed()
					&& !(input.isOuttakeButtonPressed() || input.isIntakeButtonPressed())) {
					return IntakeFSMState.MOVE_TO_HOME;
				} else {
					return IntakeFSMState.MOVE_TO_GROUND;
				}

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

/* ------------------------ Command handlers ------------------------ */

	private boolean approxEquals(double targetEncoder, double currentEncoder) {
		return Math.abs(targetEncoder - currentEncoder) < Constants.INRANGE_VALUE;
	}

	/**
	 * A Proportional controller for the pivot motor.
	 * @param currentEncoderPID The current position of the pivot motor in encoder units.
	 * @param targetEncoder The target position of the pivot motor in encoder units.
	 * @return The correct power to send to the pivot motor to achieve the target position.
	 */
	private double pid(double currentEncoderPID, double targetEncoder) {
		double correction = Constants.PID_CONSTANT_PIVOT_P
			* (targetEncoder - currentEncoderPID);
		return Math.min(Constants.MAX_TURN_SPEED,
			Math.max(Constants.MIN_TURN_SPEED, correction));
	}

	/**
	 * A PID controller for the pivot motor in autonomous mode.
	 * @param currentEncoderPID The current position of the pivot motor in encoder units.
	 * @param targetEncoder The target position of the pivot motor in encoder units.
	 * @return The correct power to send to the pivot motor to achieve the target position.
	 */
	private double pidAuto(double currentEncoderPID, double targetEncoder) {
		double correction = Constants.PID_CONSTANT_PIVOT_P_AUTO
			* (targetEncoder - currentEncoderPID);
		return Math.min(Constants.MAX_TURN_SPEED_AUTO,
			Math.max(Constants.MIN_TURN_SPEED_AUTO, correction));
	}

	/**
	 * Checks if the intake is holding a note.
	 * @return if the intake is holding a note
	 */
	public boolean hasNote() {
		boolean isInRange = colorSensor.getProximity() >= Constants.PROXIMIIY_THRESHOLD;
		SmartDashboard.putBoolean("is close enough", isInRange);

		if (isInRange) {
			noteColorFrames++;
		} else {
			noteColorFrames = 0;
		}
		hasNote = noteColorFrames >= Constants.NOTE_FRAMES_MIN;

		return hasNote;
	}

	/**
	 * Feed a note to the shooter by setting the indexer motor to the correct power.
	 * @param rpsVelocity The power to send to the indexer motor.
	 */
	public void setIndexerMotor(float rpsVelocity) {
		indexerMotor.setControl(mVoltage.withVelocity(-rpsVelocity));
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in MOVE_TO_HOME.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleMoveHomeState(TeleopInput input) {
		if (hasNote) {
			led.greenLight(false);
		} else {
			led.purpleLight();
		}

		pivotMotor.set(pid(throughBore.getDistance(), Constants.HOME_ENCODER_COUNT));
		intakeMotor.set(0);
		indexerMotor.set(0);
	}

	/**
	 * Handle behavior in MOVE_TO_GROUND.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleMoveGroundState(TeleopInput input) {
		led.orangeLight(false);

		pivotMotor.set(pid(throughBore.getDistance(), Constants.GROUND_ENCODER_COUNT));
		intakeMotor.set(0);
		indexerMotor.set(0);
	}

	/**
	 * Handle behavior in INTAKING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIntakingState(TeleopInput input) {
		if (!hasNote) {
			led.yellowLight(true);
		} else {
			led.greenLight(true);
		}

		pivotMotor.set(pid(throughBore.getDistance(), Constants.GROUND_ENCODER_COUNT));

		if (hasNote) {
			indexerMotor.set(0);
			intakeMotor.set(0);
			input.mechRightRumble(Constants.SOFT_RUMBLE);
		} else {
			indexerMotor.setControl(mVoltage.withVelocity(-Constants.INTAKE_VELOCITY/2));
			intakeMotor.setControl(mVoltage.withVelocity(Constants.INTAKE_VELOCITY));
			input.mechRightRumble(0);
		}
	}

	/**
	 * Handle behavior in OUTTAKING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOuttakingState(TeleopInput input) {
		if (hasNote) {
			led.orangeLight(false);
			input.mechLeftRumble(0);
		} else {
			led.redLight(false);
			input.mechLeftRumble(Constants.SOFT_RUMBLE);
		}

		pivotMotor.set(pid(throughBore.getDistance(), Constants.GROUND_ENCODER_COUNT));

		intakeMotor.setControl(mVoltage.withVelocity(+Constants.OUTTAKE_VELOCITY));
		indexerMotor.setControl(mVoltage.withVelocity(-Constants.OUTTAKE_VELOCITY));
	}

	/**
	 * Set the hasNote state variable, and return the new state.
	 * @param setHasNote the new state of the hasNote variable
	 * @return the new state of the hasNote variable
	 */
	public boolean setHasNote(boolean setHasNote) {
		hasNote = setHasNote;
		return hasNote;
	}

	/**
	 * Handle behavior in FEED_SHOOTER.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleFeedShooterState(TeleopInput input) {
		led.blueLight();
		input.mechBothRumble(Constants.HARD_RUMBLE);

		pivotMotor.set(pid(throughBore.getDistance(), Constants.HOME_ENCODER_COUNT));
		intakeMotor.set(0);
		indexerMotor.setControl(mVoltage.withVelocity(
			-Constants.FEED_SHOOTER_VELOCITY));
	}

	/**
	 * Performs action for auto Move to Ground.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoMoveGround() {
		led.orangeLight(false);
		pivotMotor.set(pidAuto(throughBore.getDistance(), Constants.GROUND_ENCODER_COUNT));
		intakeMotor.set(0);
		indexerMotor.set(0);

		return approxEquals(throughBore.getDistance(), Constants.GROUND_ENCODER_COUNT);
	}

	/**
	 * Performs action for auto Move to Home.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoMoveHome() {
		if (hasNote) {
			led.rainbow();
		} else {
			led.orangeLight(false);
		}

		pivotMotor.set(pidAuto(throughBore.getDistance(), Constants.HOME_ENCODER_COUNT));
		return approxEquals(throughBore.getDistance(), Constants.HOME_ENCODER_COUNT);
	}

	/**
	 * Performs action for auto Feed Shooter.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoFeedShooter() {
		if (timer.get() == 0) {
			timer.start();
		}
		pivotMotor.set(pid(throughBore.getDistance(), Constants.HOME_ENCODER_COUNT));
		if (timer.get() > Constants.AUTO_SHOOTING_SECS) {
			intakeMotor.set(0);
			indexerMotor.set(0);
			timer.stop();
			timer.reset();
			return true;
		} else {
			intakeMotor.set(0);
			indexerMotor.setControl(mVoltage.withVelocity(
				-Constants.FEED_SHOOTER_VELOCITY));
			return false;
		}
	}

	/**
	 * Performs action for auto Intake.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoIntake() {
		intakeMotor.setControl(mVoltage.withVelocity(Constants.INTAKE_VELOCITY));
		indexerMotor.setControl(mVoltage.withVelocity(-Constants.INTAKE_VELOCITY));
		pivotMotor.set(0);
		return hasNote();
	}

	/*------------------------- COMMAND CLASSES -------------------------- */

	public class IntakeCommand extends Command {
		private Timer timerSub;

		/**
		 * Initializes a new IntakeCommand.
		*/
		public IntakeCommand() {
			timerSub = new Timer();
		}

		/**
		 * Called when the command is initially scheduled.
		 *
		 * Sets LED to orange and starts timer.
		 */
		@Override
		public void initialize() {
			led.orangeLight(false);
			timerSub.start();
		}

		/**
		 * Called once the command ends or is interrupted.
		 *
		 * Sets all intake motors to 0 power and resets the timer.
		 * @param interrupted Whether the command was interrupted
		 */
		@Override
		public void end(boolean interrupted) {
			intakeMotor.set(0);
			indexerMotor.set(0);

			timerSub.stop();
			timerSub.reset();
		}

		/**
		 * Returns true when the command should end.
		 */
		@Override
		public boolean isFinished() {
			return handleAutoIntake() || timerSub.get() >= 2.0;
		}
	}

	public class PivotToGroundCommand extends Command {

		private Timer timerSub;

		/**
		 * Initializes a new PivotToGroundCommand.
		 */
		public PivotToGroundCommand() {
			timerSub = new Timer();
		}

		/**
		 * Called when the command is initially scheduled.
		 */
		public void initialize() {
			led.orangeLight(false);
			timerSub.start();
		}

		/**
		 * Called every time the scheduler runs while the command is scheduled.
		 */
		@Override
		public void execute() {
			System.out.println("htg");
		}

		/**
		 * Called once the command ends or is interrupted.
		 */
		@Override
		public void end(boolean interrupted) {
			pivotMotor.set(0);
			timerSub.stop();
			timerSub.reset();
		}

		/**
		 * Returns true when the command should end.
		 * @return if the action is completed
		 */
		@Override
		public boolean isFinished() {
			return handleAutoMoveGround();
		}
	}

	public class PivotToHomeCommand extends Command {

		private Timer timerSub;

		/**
		 * Initializes a new PivotToHomeCommand.
		 */
		public PivotToHomeCommand() {
			timerSub = new Timer();
		}

		/**
		 * Called when the command is initially scheduled.
		 */
		public void initialize() {
			timerSub.start();
		}

		/**
		 * Called every time the scheduler runs while the command is scheduled.
		 */
		@Override
		public void execute() {
			if (hasNote) {
				led.rainbow();
			}

			System.out.println("gth");
		}

		/**
		 * Called once the command ends or is interrupted.
		 */
		@Override
		public void end(boolean interrupted) {
			intakeMotor.set(0);
			indexerMotor.set(0);
			pivotMotor.set(0);

			timerSub.stop();
			timerSub.reset();
		}

		@Override
		public boolean isFinished() {
			return handleAutoMoveHome();
		}
	}
}

