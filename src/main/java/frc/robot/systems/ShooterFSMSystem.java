package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
// Third party Hardware Imports
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.Constants;

public class ShooterFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum ShooterFSMState {
		IDLE_STOP,
		REV_UP_SHOOTER,
	}

	private final MotionMagicVelocityVoltage mVoltage = new MotionMagicVelocityVoltage(0);

	/* ======================== Private variables ======================== */
	private ShooterFSMState currentState;
	private Timer timer = new Timer();
	private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
	private Slot0Configs slot0Configs = talonFXConfigs.Slot0;
	private MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
	private StatusCode statusCode = StatusCode.StatusCodeNotInitialized;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private TalonFX shooterLeftMotor;
	private TalonFX shooterRightMotor;

	/* ======================== Constructor ======================== */
	/**
	 * Create PivotFSM and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ShooterFSMSystem() {
		shooterLeftMotor = new TalonFX(HardwareMap.LEFT_SHOOTER_MOTOR_ID);
		shooterLeftMotor.setNeutralMode(NeutralModeValue.Coast);

		shooterRightMotor = new TalonFX(HardwareMap.RIGHT_SHOOTER_MOTOR_ID);
		shooterLeftMotor.setNeutralMode(NeutralModeValue.Coast);

		// Add 0.25 V output to overcome static friction
		slot0Configs.kS = Constants.MM_CONSTANT_S;
		// A velocity target of 1 rps results in 0.114 V output
		slot0Configs.kV = Constants.MM_CONSTANT_V;
		// An acceleration of 1 rps/s requires 0.01 V output
		slot0Configs.kA = Constants.MM_CONSTANT_A;

		slot0Configs.kP = Constants.MM_CONSTANT_P;
		slot0Configs.kI = Constants.MM_CONSTANT_I;
		slot0Configs.kD = Constants.MM_CONSTANT_D;

		motionMagicConfigs.MotionMagicAcceleration = Constants.CONFIG_CONSTANT_A;
		// Target acceleration 400 rps/s (0.25 seconds to max)
		motionMagicConfigs.MotionMagicJerk = Constants.CONFIG_CONSTANT_J;
		// Target jerk of 4000 rps/s/s (0.1 seconds)

		statusCode = shooterLeftMotor.getConfigurator().apply(talonFXConfigs);
		statusCode = shooterRightMotor.getConfigurator().apply(talonFXConfigs);

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public ShooterFSMState getCurrentState() {
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
		currentState = ShooterFSMState.IDLE_STOP;
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

		SmartDashboard.putString("TeleOP STATE", currentState.toString());
		SmartDashboard.putString("Current State", getCurrentState().toString());
		SmartDashboard.putNumber("Motor Velocity Left",
			shooterLeftMotor.getVelocity().getValueAsDouble());
		SmartDashboard.putNumber("Motor Velocity Right",
				shooterRightMotor.getVelocity().getValueAsDouble());

		switch (currentState) {
			case IDLE_STOP:
				handleIdleState(input);
				break;
			case REV_UP_SHOOTER:
				handleRevShooterState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		currentState = nextState(input);
	}

	///**
	// * Performs specific action based on the autoState passed in.
	// * @param autoState autoState that the subsystem executes.
	// * @return if the action carried out in this state has finished executing
	// */
	// public boolean updateAutonomous(AutoFSMState autoState) {
	// 	switch (autoState) {
	// 		case NOTE1:
	// 			return handleAutoMoveGround() && handleAutoIntake();
	// 		case NOTE2:
	// 			return handleAutoMoveGround() && handleAutoIntake();
	// 		case NOTE3:
	// 			return handleAutoMoveGround() && handleAutoIntake();
	// 		case NOTE4:
	// 			return handleAutoMoveGround() && handleAutoIntake();
	// 		case NOTE5:
	// 			return handleAutoMoveGround() && handleAutoIntake();
	// 		case NOTE6:
	// 			return handleAutoMoveGround() && handleAutoIntake();
	// 		case NOTE7:
	// 			return handleAutoMoveGround() && handleAutoIntake();
	// 		case NOTE8:
	// 			return handleAutoMoveGround() && handleAutoIntake();
	// 		case SPEAKER:
	// 			return handleAutoMoveShooter() & handleAutoRev();
	// 		case SHOOT:
	// 			return handleAutoShoot();
	// 		case SHOOT_PRELOADED:
	// 			return handleAutoShootPreloaded();
	// 		default:
	// 			return true;
	// 	}
	// }

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
	private ShooterFSMState nextState(TeleopInput input) {
		switch (currentState) {
			case IDLE_STOP:
				if (input == null) {
					return ShooterFSMState.IDLE_STOP;
				}

				if (input.isRevButtonPressed()) {
					return ShooterFSMState.REV_UP_SHOOTER;
				} else {
					return ShooterFSMState.IDLE_STOP;
				}

			case REV_UP_SHOOTER:
				if (input.isRevButtonPressed()) {
					return ShooterFSMState.REV_UP_SHOOTER;
				} else {
					return ShooterFSMState.IDLE_STOP;
				}
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ Command handlers ------------------------ */

	/* ---------------------------- FSM State Handlers ---------------------------- */

	/**
	 * Handles the Rev up State of the Shooter.
	 * @param input action input by driver on the mech controller
	 */
	public void handleIdleState(TeleopInput input) {
		shooterLeftMotor.setControl(mVoltage.withVelocity(0));
		shooterRightMotor.setControl(mVoltage.withVelocity(0));

		input.mechLeftRumble(0);
	}

	/**
	 * Handles the Rev up State of the Shooter.
	 * @param input action input by driver on the mech controller
	 */
	public void handleRevShooterState(TeleopInput input) {
		shooterLeftMotor.setControl(mVoltage.withVelocity(-Constants.SHOOT_VELOCITY));
		shooterRightMotor.setControl(mVoltage.withVelocity(Constants.SHOOT_VELOCITY));

		input.mechBothRumble(Constants.SOFT_RUMBLE);
	}

	/**
	 * Handles the Auto Rev state of the MBR Mech.
	 * @return if the action is completed
	 */
	public boolean handleAutoRev() {
		// shooterLeftMotor.set(-MechConstants.SHOOTING_POWER); // dont forget the "-" sign
		// shooterRightMotor.set(MechConstants.SHOOTING_POWER);
		shooterLeftMotor.setControl(mVoltage.withVelocity(-Constants.SHOOT_VELOCITY));
		shooterRightMotor.setControl(mVoltage.withVelocity(Constants.SHOOT_VELOCITY));
		return true;
	}

	/**
	 * Handles the Auto Shoot state for a preloaded note.
	 * @return if the action is completed
	 */
	public boolean handleAutoShootPreloaded() {
		if (timer.get() == 0) {
			timer.start();
		}

		if (timer.get() < Constants.AUTO_PRELOAD_REVVING_SECS) {
			shooterLeftMotor.setControl(mVoltage.withVelocity(
				-Constants.SHOOT_VELOCITY));
			shooterRightMotor.setControl(mVoltage.withVelocity(
				Constants.SHOOT_VELOCITY));
			return false;
		} else if (timer.get() < Constants.AUTO_PRELOAD_SHOOTING_SECS) {
			shooterLeftMotor.setControl(mVoltage.withVelocity(
				-Constants.SHOOT_VELOCITY));
			shooterRightMotor.setControl(mVoltage.withVelocity(
				Constants.SHOOT_VELOCITY));
			return false;
		} else {
			shooterLeftMotor.setControl(mVoltage.withVelocity(0));
			shooterRightMotor.setControl(mVoltage.withVelocity(0));
			timer.stop();
			timer.reset();
			return true;
		}
	}

	/*-------------------------- COMMAND CLASSES -------------------------- */

	public class ShootPreloadedCommand extends Command {

		private Timer timerSub;
		private IntakeFSMSystem intakeFSM;

		/**
		 * Creates a new ShootPreloadedCommand.
		 * @param intake the IntakeFSMSystem
		*/
		public ShootPreloadedCommand(IntakeFSMSystem intake) {
			timerSub = new Timer();
			this.intakeFSM = intake;
		}

		/**
		 * Called when the command is initially scheduled.
		*/
		@Override
		public void initialize() {
			timerSub.start();
		}

		@Override
		public void execute() {
			if (timerSub.get() < Constants.AUTO_PRELOAD_REVVING_SECS) {
				shooterLeftMotor.setControl(mVoltage.withVelocity(
					-Constants.SHOOT_VELOCITY));
				shooterRightMotor.setControl(mVoltage.withVelocity(
					Constants.SHOOT_VELOCITY));
				intakeFSM.setIndexerMotor(0);
			} else if (timerSub.get() < Constants.AUTO_PRELOAD_SHOOTING_SECS) {
				shooterLeftMotor.setControl(mVoltage.withVelocity(
					-Constants.SHOOT_VELOCITY));
				shooterRightMotor.setControl(mVoltage.withVelocity(
					Constants.SHOOT_VELOCITY));
				intakeFSM.setIndexerMotor(Constants.FEED_SHOOTER_VELOCITY);
			}
		}

		@Override
		public void end(boolean interrupted) {
			shooterLeftMotor.setControl(mVoltage.withVelocity(0));
			shooterRightMotor.setControl(mVoltage.withVelocity(0));
			intakeFSM.setIndexerMotor(0);
			intakeFSM.setHasNote(false);

			timerSub.stop();
			timerSub.reset();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return timerSub.get() >= Constants.AUTO_PRELOAD_SHOOTING_SECS;
		}
	}

	public class ShootNoteCommand extends Command {

		private Timer timerSub;
		private IntakeFSMSystem intakeFSM;

		/**
		 * Initializes a new ShootNoteCommand.
		 * @param intake the IntakeFSMSystem
		 */
		public ShootNoteCommand(IntakeFSMSystem intake) {
			timerSub = new Timer();
			this.intakeFSM = intake;
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			timerSub.start();
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			if (timerSub.get() < Constants.AUTO_SHOOTING_SECS) {
				shooterLeftMotor.setControl(mVoltage.withVelocity(
					-Constants.SHOOT_VELOCITY));
				shooterRightMotor.setControl(mVoltage.withVelocity(
					Constants.SHOOT_VELOCITY));
				intakeFSM.setIndexerMotor(Constants.FEED_SHOOTER_VELOCITY);
			}
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			shooterLeftMotor.setControl(mVoltage.withVelocity(0));
			shooterRightMotor.setControl(mVoltage.withVelocity(0));
			intakeFSM.setIndexerMotor(0);

			timerSub.stop();
			timerSub.reset();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return timerSub.get() >= Constants.AUTO_SHOOTING_SECS;
		}
	}

	public class RevCommand extends Command {
		private Timer timerSub;
		private IntakeFSMSystem intakeFSM;

		/**
		 * Initializes a new RevCommand.
		 * @param intake the IntakeFSMSystem
		 */
		public RevCommand(IntakeFSMSystem intake) {
			timerSub = new Timer();
			this.intakeFSM = intake;
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			timerSub.start();
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			if (timerSub.get() < Constants.AUTO_REVVING_SECS) {
				shooterLeftMotor.setControl(mVoltage.withVelocity(
					-Constants.SHOOT_VELOCITY));
				shooterRightMotor.setControl(mVoltage.withVelocity(
					Constants.SHOOT_VELOCITY));
				intakeFSM.setIndexerMotor(0);
			}
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			shooterLeftMotor.setControl(mVoltage.withVelocity(0));
			shooterRightMotor.setControl(mVoltage.withVelocity(0));
			intakeFSM.setIndexerMotor(0);

			timerSub.stop();
			timerSub.reset();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return timerSub.get() >= Constants.AUTO_REVVING_SECS;
		}
	}
}
