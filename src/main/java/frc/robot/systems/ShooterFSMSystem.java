package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.LED;
import frc.robot.MechConstants;

public class ShooterFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum ShooterFSMState {
		IDLE_STOP,
		REV_UP_SHOOTER,
	}

	/* ======================== Private variables ======================== */
	private ShooterFSMState currentState;
	private LED led = new LED();
	private Timer timer = new Timer();

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax shooterTopMotor;
	private CANSparkMax shooterBottomMotor;

	/* ======================== Constructor ======================== */
	/**
	 * Create PivotFSM and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ShooterFSMSystem() {
		shooterTopMotor = new CANSparkMax(HardwareMap.TOP_SHOOTER_CAN_ID,
						CANSparkMax.MotorType.kBrushless);
		shooterTopMotor.setIdleMode(IdleMode.kCoast);

		shooterBottomMotor = new CANSparkMax(HardwareMap.BOTTOM_SHOOTER_CAN_ID,
						CANSparkMax.MotorType.kBrushless);
		shooterBottomMotor.setIdleMode(IdleMode.kCoast);

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
		led.greenLight(false);
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
				}
				return ShooterFSMState.IDLE_STOP;
			case REV_UP_SHOOTER:
				if (input.isRevButtonPressed()) {
					return ShooterFSMState.REV_UP_SHOOTER;
				}

				if (!input.isRevButtonPressed()) {
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
		led.purpleLight();
		shooterTopMotor.set(0);
		shooterBottomMotor.set(0);
	}

	/**
	 * Handles the Rev up State of the Shooter.
	 * @param input action input by driver on the mech controller
	 */
	public void handleRevShooterState(TeleopInput input) {
		led.blueLight();
		shooterTopMotor.set(-MechConstants.SHOOTING_POWER); // dont forget the "-" sign
		shooterBottomMotor.set(MechConstants.SHOOTING_POWER);
	}

	/**
	 * Handles the Auto Rev state of the MBR Mech.
	 * @return if the action is completed
	 */
	public boolean handleAutoRev() {
		shooterTopMotor.set(-MechConstants.SHOOTING_POWER); // dont forget the "-" sign
		shooterBottomMotor.set(MechConstants.SHOOTING_POWER);
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

		if (timer.get() < MechConstants.AUTO_PRELOAD_REVVING_TIME) {
			shooterTopMotor.set(-MechConstants.SHOOTING_POWER); // dont forget the "-" sign
			shooterBottomMotor.set(MechConstants.SHOOTING_POWER);
			return false;
		} else if (timer.get() < MechConstants.AUTO_PRELOAD_SHOOTING_TIME) {
			shooterTopMotor.set(-MechConstants.SHOOTING_POWER); // dont forget the "-" sign
			shooterBottomMotor.set(MechConstants.SHOOTING_POWER);
			return false;
		} else {
			shooterTopMotor.set(0);
			shooterBottomMotor.set(0);
			timer.stop();
			timer.reset();
			return true;
		}
	}
}
