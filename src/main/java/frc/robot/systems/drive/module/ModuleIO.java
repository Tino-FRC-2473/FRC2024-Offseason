package frc.robot.systems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

	@AutoLog
	public class ModuleIOInfo {
		public boolean connected = false;

		public double drivePosition = 0.0; // rad
		public double driveVelocity = 0.0; // rad/s

		public double driveAppliedVolts = 0.0;
		public double driveCurrentAmps = 0.0;

		//this is generated from the CANCoder
		public Rotation2d turnAbsolutePosition = new Rotation2d();
		//this is using the talonfx encoder val
		public Rotation2d turnRelativePosition = new Rotation2d();
		public double turnVelocity = 0.0;

		public double turnAppliedVolts = 0.0;
		public double turnCurrentAmps = 0.0;
	}

	/**
	 * Update all the values of the logged values of the module through a ModuleIOInfo object.
	 * @param moduleInfo
	 */
	default void updateInputs(ModuleIOInfo moduleInfo) {
	}

	/**
	 * Run the drive motor at the specified voltage.
	 * @param volts
	*/
	default void setDriveVoltage(double volts) {
	}

	/**
	 * Run the turn motor at the specified voltage.
	 * @param volts
	 */
	default void setTurnVoltage(double volts) {
	}

	/**
	 * Set whether the drive motor is on brake mode or not.
	 * @param enable
	*/
	default void setDriveBrakeMode(boolean enable) {
	}

	/**
	 * Set whether the turn motor is on brake mode or not.
	 * @param enable
	 */
	default void setTurnBrakeMode(boolean enable) {
	}

	/**
	 * Set the name of the module.
	 * @param moduleName
	 */
	default void setModuleName(String moduleName) {
	}

	/**
	 * Get the shorthanded logging name of the module.
	 * @return Name of the module
	 */
	default String getModuleName() {
		return "";
	}

	/** Reset the encoder position values. */
	default void resetEncoders() {
	}

}
