package frc.robot.systems.drive.module;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveConstants.ModuleConstants;

import org.littletonrobotics.junction.Logger;


public class Module {
	private final ModuleIO io;

	//this is generated when you put the @AutoLog annotation on the ModuleInfo class
	private ModuleIOInfoAutoLogged inputs;

	private final SimpleMotorFeedforward driveFeedforward;
	private final PIDController driveFeedback;
	private final PIDController turnFeedback;
	private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
	private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop
	private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute

	/**
	 * Creates a Module object that the FSMs interact with.
	 * @param moduleIO the IO of the module that the Module class interacts with
	 */
	public Module(ModuleIO moduleIO) {
		io = moduleIO;
		inputs = new ModuleIOInfoAutoLogged(io.getModuleName());

		driveFeedforward = new SimpleMotorFeedforward(
			ModuleConstants.DRIVING_FF_KS,
			ModuleConstants.DRIVING_FF_KV);
		driveFeedback = new PIDController(
			ModuleConstants.DRIVING_P,
			ModuleConstants.DRIVING_I,
			ModuleConstants.DRIVING_D);
		turnFeedback = new PIDController(
			ModuleConstants.TURNING_P,
			ModuleConstants.TURNING_I,
			ModuleConstants.TURNING_D);

		turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
		setBrakeMode(true);
	}

	/** Logger to process and update inputs to be called every periodic on the MAIN thread. */
	public void processInputs() {
		io.updateInputs(inputs); //can it be in the main thread? unconfirmed
		Logger.processInputs("Drive Module " + io.getModuleName() + " Inputs", inputs);
	}

	/**
	 * Set the desired state for the module based on the setpoint.
	 * @param desiredState the desired state for the module
	 * @return the optimized swerve state that the closed loop correction is set on
	 */
	public SwerveModuleState setDesiredState(SwerveModuleState desiredState) {
		// Optimize state based on current angle
		SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getAngle());

		// Update setpoints, call setClosedLoopCorrection after
		angleSetpoint = optimizedState.angle;
		speedSetpoint = optimizedState.speedMetersPerSecond;

		//Triggers closed loop correction cycles
		setCloosedLoopCorrection();

		return optimizedState;
	}

	/**
	 * Voltage based closed loop turn control
	 * based on the angle and speed setpoint of the desired state.
	 */
	private void setCloosedLoopCorrection() {
		// Run closed loop turn control
		io.setTurnVoltage(
			turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

		// Scale velocity based on turn error
		// - vector projection of speedSetpoint on error deviation (in radians)
		double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getPositionError());

		// Run closed loop drive control
		double velocity = adjustSpeedSetpoint / ModuleConstants.WHEEL_RADIUS;
		io.setDriveVoltage(
			driveFeedforward.calculate(velocity)
				+ driveFeedback.calculate(inputs.getDriveVelocity(), velocity));
	}

	/** Reset drive and turn encoders. */
	public void resetEncoders() {
		//resets relative turn encoder
		turnRelativeOffset = inputs.getTurnAbsolutePosition()
			.minus(inputs.getTurnRelativePosition());
		io.resetEncoders();
	}

	/**
	 * Runs the module with the specified voltage while controlling to zero degrees.
	 * @param volts
	 */
	public void runCharacterization(double volts) {
		// Closed loop turn control
		angleSetpoint = new Rotation2d();

		// Open loop drive control
		io.setDriveVoltage(volts);
		speedSetpoint = null;
	}

	/** Disables all outputs to motors. */
	public void stop() {
		io.setTurnVoltage(0.0);
		io.setDriveVoltage(0.0);

		// Disable closed loop control for turn and drive
		angleSetpoint = null;
		speedSetpoint = null;
	}

	/**
	 * Sets whether brake mode is enabled.
	 * @param enabled
	 */
	public void setBrakeMode(boolean enabled) {
		io.setDriveBrakeMode(enabled);
		io.setTurnBrakeMode(enabled);
	}

	/**
	 * Returns the current turn angle of the module.
	 * @return current turn angle of module in radians.
	 */
	public Rotation2d getAngle() {
		return inputs.getTurnRelativePosition().plus(
			turnRelativeOffset != null ? turnRelativeOffset : new Rotation2d()
		);
	}

	/**
	 * Returns the current drive position of the module.
	 * @return Drive position in m
	 */
	public double getPositionMeters() {
		return inputs.getDrivePosition() * ModuleConstants.WHEEL_RADIUS;
	}

	/**
	 * Returns the current drive velocity of the module.
	 * @return Drive velocity in m/s
	 */
	public double getVelocityMetersPerSec() {
		return inputs.getDriveVelocity() * ModuleConstants.WHEEL_RADIUS;
	}

	/**
	 * Returns the module position.
	 * @return module position with turn angle and drive position.
	 */
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getPositionMeters(), getAngle());
	}

	/**
	 * Returns the module state.
	 * @return SwerveModuleState with turn angle and drive velocity.
	 */
	public SwerveModuleState getState() {
		return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
	}

	/**
	 * Returns the drive velocity in radians/sec.
	 * @return drive motor velocity
	*/
	public double getCharacterizationVelocity() {
		return inputs.getDriveVelocity();
	}
}
