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
	private TalonFX frontIndexMotor;
	private TalonFX backIndexMotor;
	private TalonFX topIntakeMotor;
	private TalonFX bottomIntakeMotor;
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
		frontIndexMotor = new TalonFX(HardwareMap.FRONT_INDEXER_MOTOR_ID);
		frontIndexMotor.setNeutralMode(NeutralModeValue.Brake);

		backIndexMotor = new TalonFX(HardwareMap.BACK_INDEXER_MOTOR_ID);
		backIndexMotor.setNeutralMode(NeutralModeValue.Brake);

		pivotMotor = new TalonFX(HardwareMap.PIVOT_MOTOR_ID);
		pivotMotor.setNeutralMode(NeutralModeValue.Brake);

		topIntakeMotor = new TalonFX(HardwareMap.TOP_INTAKE_MOTOR_ID);
		topIntakeMotor.setNeutralMode(NeutralModeValue.Brake);

		bottomIntakeMotor = new TalonFX(HardwareMap.BOTTOM_INTAKE_MOTOR_ID);
		bottomIntakeMotor.setNeutralMode(NeutralModeValue.Brake);

		throughBore = new Encoder(0, 1);
		throughBore.reset();

		colorSensor = new ColorSensorV3(Port.kOnboard);
		led = new LED();

		// Add 0.25 V output to overcome static friction
		slot0Configs.kS = Constants.MM_CONSTANT_S;
		// A velocity target of 1 rps results in 0.114 V output
		slot0Configs.kV = Constants.MM_CONSTANT_V;
		// An acceleration of 1 rps/s requires 0.01 V output
		slot0Configs.kA = Constants.MM_CONSTANT_A;

		slot0Configs.kP = Constants.MM_CONSTANT_P;
		slot0Configs.kI = Constants.MM_CONSTANT_I;
		slot0Configs.kD = 0;

		motionMagicConfigs.MotionMagicAcceleration = Constants.CONFIG_CONSTANT_A;
		// Target acceleration 400 rps/s (0.25 seconds to max)
		motionMagicConfigs.MotionMagicJerk = Constants.CONFIG_CONSTANT_J;
		// Target jerk of 4000 rps/s/s (0.1 seconds)

		statusCode = frontIndexMotor.getConfigurator().apply(talonFXConfigs);
		statusCode = backIndexMotor.getConfigurator().apply(talonFXConfigs);
		statusCode = topIntakeMotor.getConfigurator().apply(talonFXConfigs);
		statusCode = bottomIntakeMotor.getConfigurator().apply(talonFXConfigs);

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

		SmartDashboard.putString("CURRENT STATE", currentState.toString());
		SmartDashboard.putNumber("Back Indexer Velocity",
			backIndexMotor.getVelocity().getValueAsDouble());
		SmartDashboard.putNumber("Front Indexer Velocity",
			frontIndexMotor.getVelocity().getValueAsDouble());
		SmartDashboard.putBoolean("HASNOTE", hasNote);
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
				if (inRange(throughBore.getDistance(), Constants.GROUND_ENCODER_ROTATIONS)) {
					if (input.isIntakeButtonPressed() && !input.isOuttakeButtonPressed()
						&& !hasNote && !input.isShootButtonPressed()) {
						return IntakeFSMState.INTAKING;
					} else if (input.isOuttakeButtonPressed() && !input.isIntakeButtonPressed()
								&& input.isShootButtonPressed()) {
						return IntakeFSMState.OUTTAKING;
					}
				} else if (!inRange(throughBore.getDistance(),
					Constants.GROUND_ENCODER_ROTATIONS)) {
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
				if (inRange(throughBore.getDistance(), Constants.GROUND_ENCODER_ROTATIONS)
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
				if (inRange(throughBore.getDistance(), Constants.GROUND_ENCODER_ROTATIONS)
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
		return Math.abs(a - b) < Constants.INRANGE_VALUE; //EXPERIMENTAL
	}

	/**
	 * A PID controller for the pivot motor.
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

		noteColorFrames = isInRange ? (noteColorFrames + 1) : 0;
		hasNote = noteColorFrames >= Constants.NOTE_FRAMES_MIN;

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

		pivotMotor.set(pid(throughBore.getDistance(), Constants.HOME_ENCODER_ROTATIONS));
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

		pivotMotor.set(pid(throughBore.getDistance(), Constants.GROUND_ENCODER_ROTATIONS));
		topIntakeMotor.set(0);
		bottomIntakeMotor.set(0);
		backIndexMotor.set(0);
		frontIndexMotor.set(0);
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

		pivotMotor.set(pid(throughBore.getDistance(), Constants.GROUND_ENCODER_ROTATIONS));
		topIntakeMotor.setControl(Constants.MM_VOLTAGE.withVelocity(Constants.INTAKE_VELOCITY));
		bottomIntakeMotor.setControl(Constants.MM_VOLTAGE.withVelocity(Constants.INTAKE_VELOCITY));
		backIndexMotor.setControl(Constants.MM_VOLTAGE.withVelocity(-Constants.INTAKE_VELOCITY));
		// don't forget the "-"" sign
		frontIndexMotor.setControl(Constants.MM_VOLTAGE.withVelocity(Constants.INTAKE_VELOCITY));
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

		pivotMotor.set(pid(throughBore.getDistance(), Constants.GROUND_ENCODER_ROTATIONS));
		bottomIntakeMotor.set(0);
		topIntakeMotor.set(0);
		backIndexMotor.setControl(Constants.MM_VOLTAGE.withVelocity(-Constants.OUTTAKE_VELOCITY));
		// don't forget the "-"" sign
		frontIndexMotor.setControl(Constants.MM_VOLTAGE.withVelocity(Constants.OUTTAKE_VELOCITY));
	}

	/**
	 * Handle behavior in MOVE_TO_HOME.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleFeedShooterState(TeleopInput input) {
		led.blueLight();

		pivotMotor.set(pid(throughBore.getDistance(), Constants.HOME_ENCODER_ROTATIONS));
		topIntakeMotor.set(0);
		bottomIntakeMotor.set(0);
		backIndexMotor.setControl(Constants.MM_VOLTAGE.withVelocity(
			-Constants.FEED_SHOOTER_VELOCITY));
		// don't forget the "-"" sign
		frontIndexMotor.setControl(Constants.MM_VOLTAGE.withVelocity(
			Constants.FEED_SHOOTER_VELOCITY));
	}

	/**
	 * Performs action for auto Move to Ground.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoMoveGround() {
		led.orangeLight(false);
		pivotMotor.set(pidAuto(throughBore.getDistance(), Constants.GROUND_ENCODER_ROTATIONS));
		return inRange(throughBore.getDistance(), Constants.GROUND_ENCODER_ROTATIONS);
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

		pivotMotor.set(pidAuto(throughBore.getDistance(), Constants.HOME_ENCODER_ROTATIONS));
		return inRange(throughBore.getDistance(), Constants.HOME_ENCODER_ROTATIONS);
	}

	/**
	 * Performs action for auto STATE3.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoOuttake() {
		if (timer.get() == 0) {
			timer.start();
		}
		pivotMotor.set(pid(throughBore.getDistance(), Constants.HOME_ENCODER_ROTATIONS));
		if (timer.get() > Constants.AUTO_SHOOTING_TIME) {
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
			backIndexMotor.setControl(Constants.MM_VOLTAGE.withVelocity(
				-Constants.FEED_SHOOTER_VELOCITY));
			// don't forget the "-"" sign
			frontIndexMotor.setControl(Constants.MM_VOLTAGE.withVelocity(
				Constants.FEED_SHOOTER_VELOCITY));
			return false;
		}
	}

	/**
	 * Performs action for auto Intake.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoIntake() {
		bottomIntakeMotor.setControl(Constants.MM_VOLTAGE.withVelocity(Constants.INTAKE_VELOCITY));
		topIntakeMotor.setControl(Constants.MM_VOLTAGE.withVelocity(Constants.INTAKE_VELOCITY));
		frontIndexMotor.setControl(Constants.MM_VOLTAGE.withVelocity(Constants.INTAKE_VELOCITY));
		backIndexMotor.setControl(Constants.MM_VOLTAGE.withVelocity(-Constants.INTAKE_VELOCITY));
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
			topIntakeMotor.set(0);
			bottomIntakeMotor.set(0);
			frontIndexMotor.set(0);
			backIndexMotor.set(0);
			timerSub.stop();
			timerSub.reset();
		}

		/**
		 * Returns true when the command should end.
		 */
		@Override
		public boolean isFinished() {
			return handleAutoIntake();
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
		 * Called every time the scheduler runs while the command is scheduled.
		 */
		@Override
		public void execute() {
			led.orangeLight(false);
			if (handleAutoMoveGround()) {
				timerSub.start();
			}

			System.out.println("ptg");
		}

		/**
		 * Called once the command ends or is interrupted.
		 */
		@Override
		public void end(boolean interrupted) {
			timerSub.stop();
			timerSub.reset();
		}

		/**
		 * Returns true when the command should end.
		 * @return if the action is completed
		 */
		@Override
		public boolean isFinished() {
			return handleAutoMoveGround() && timerSub.get() > Constants.AUTO_PIVOT_TIMER;
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
		 * Called every time the scheduler runs while the command is scheduled.
		 */
		@Override
		public void execute() {
			if (hasNote) {
				led.rainbow();
			}

			if (handleAutoMoveHome()) {
				timerSub.start();
			}
		}

		/**
		 * Called once the command ends or is interrupted.
		 */
		@Override
		public void end(boolean interrupted) {
			topIntakeMotor.set(0);
			bottomIntakeMotor.set(0);
			frontIndexMotor.set(0);
			backIndexMotor.set(0);
			timerSub.stop();
			timerSub.reset();
		}

		@Override
		public boolean isFinished() {
			return handleAutoMoveHome();
		}
	}

	public class OuttakeNoteCommand extends Command {

		private Timer timerSub;

		/**
		 * Initializes a new OuttakeNoteCommand.
		 */
		public OuttakeNoteCommand() {
			timerSub = new Timer();
		}

		/**
		 * Called when the command is initially scheduled.
		 */
		@Override
		public void initialize() {
			led.blueLight();
			timerSub.start();
		}

		/**
		 * Called every time the scheduler runs while the command is scheduled.
		 */
		@Override
		public void execute() {
			handleAutoOuttake();
			System.out.println("OUTTAKING" + timerSub.get());
		}

		/**
		 * Returns true when the command should end.
		 */
		@Override
		public boolean isFinished() {
			return handleAutoOuttake() || timerSub.get() >= Constants.OUTTAKE_AUTO_TIMER;
		}

		/**
		 * Called once the command ends or is interrupted.
		 */
		@Override
		public void end(boolean interrupted) {
			timerSub.stop();
			timerSub.reset();

			topIntakeMotor.set(0);
			bottomIntakeMotor.set(0);
			frontIndexMotor.set(0);
			backIndexMotor.set(0);
			hasNote = false;
		}
	}

	public class OuttakePreloadedCommand extends Command {

		private Timer timerSub;

		/**
		 * Initializes a new OuttakePreloadedCommand.
		 */
		public OuttakePreloadedCommand() {
			timerSub = new Timer();
		}

		@Override
		public void initialize() {
			timerSub.start();
		}

		/**
		 * Called every time the scheduler runs while the command is scheduled.
		 */
		@Override
		public void execute() {
			led.rainbow();
			pivotMotor.set(pidAuto(throughBore.getDistance(),
				Constants.HOME_ENCODER_ROTATIONS));

			if (timerSub.get() < Constants.AUTO_PRELOAD_REVVING_TIME) {
				topIntakeMotor.set(0);
				bottomIntakeMotor.set(0);
				frontIndexMotor.set(0);
				backIndexMotor.set(0);
			} else if (timerSub.get() < Constants.AUTO_PRELOAD_SHOOTING_TIME) {
				topIntakeMotor.set(0);
				bottomIntakeMotor.set(0);
				frontIndexMotor.setControl(Constants.MM_VOLTAGE.withVelocity(
					Constants.INTAKE_VELOCITY));
				backIndexMotor.setControl(Constants.MM_VOLTAGE.withVelocity(
						-Constants.INTAKE_VELOCITY));
			}
		}

		/**
		 * Called once the command ends or is interrupted.
		 */
		@Override
		public void end(boolean interrupted) {
			topIntakeMotor.set(0);
			bottomIntakeMotor.set(0);
			frontIndexMotor.set(0);
			backIndexMotor.set(0);
			timerSub.stop();
			timerSub.reset();
		}

		/**
		 * Returns true when the command should end.
		 */
		@Override
		public boolean isFinished() {
			return timerSub.get() >= Constants.AUTO_PRELOAD_SHOOTING_TIME;
		}
	}
}

