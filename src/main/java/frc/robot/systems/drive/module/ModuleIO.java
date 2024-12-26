package frc.robot.systems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {

	class ModuleIOInfo {
		private boolean connected = false;

		private double drivePosition = 0.0; // rad
		private double driveVelocity = 0.0; // rad/s

		private double driveAppliedVolts = 0.0;
		private double driveCurrentAmps = 0.0;

		//this is generated from the CANCoder
		private Rotation2d turnAbsolutePosition = new Rotation2d();
		//this is using the talonfx encoder val
		private Rotation2d turnRelativePosition = new Rotation2d();
		private double turnVelocity = 0.0;

		private double turnAppliedVolts = 0.0;
		private double turnCurrentAmps = 0.0;

		/**
		 * Get the connection status.
		 * @return connected
		 */
		public boolean isConnected() {
			return this.connected;
		}

		/**
		 * Set the connection status.
		 * @param isConnected
		 */
		public void setConnected(boolean isConnected) {
			connected = isConnected;
		}

		/**
		 * Get the drive position.
		 * @return drive position
		 */
		public double getDrivePosition() {
			return drivePosition;
		}

		/**
		 * Set the drive position.
		 * @param drivePos
		 */
		public void setDrivePosition(double drivePos) {
			drivePosition = drivePos;
		}

		/**
		 * Get the drive velocity.
		 * @return drive velocity
		 */
		public double getDriveVelocity() {
			return driveVelocity;
		}

		/**
		 * Set the drive velocity.
		 * @param driveVel
		 */
		public void setDriveVelocity(double driveVel) {
			driveVelocity = driveVel;
		}

		/**
		 * Get the drive applied volts.
		 * @return drive applied volts
		 */
		public double getDriveAppliedVolts() {
			return this.driveAppliedVolts;
		}

		/**
		 * Set the drive applied volts.
		 * @param driveVolts
		 */
		public void setDriveAppliedVolts(double driveVolts) {
			this.driveAppliedVolts = driveVolts;
		}

		/**
		 * Get the drive current amps.
		 * @return drive current amps.
		 */
		public double getDriveCurrentAmps() {
			return this.driveCurrentAmps;
		}

		/**
		 * Set the drive current amps.
		 * @param driveAmps
		 */
		public void setDriveCurrentAmps(double driveAmps) {
			this.driveCurrentAmps = driveAmps;
		}

		/**
		 * Get the turn absolute position.
		 * @return turn absolute position.
		 */
		public Rotation2d getTurnAbsolutePosition() {
			return this.turnAbsolutePosition;
		}

	/**
	 * Set the turn absolute position.
	 * @param turnAbsPos
	 */
		public void setTurnAbsolutePosition(Rotation2d turnAbsPos) {
			this.turnAbsolutePosition = turnAbsPos;
		}

		/**
		 * Get the turn relative position.
		 * @return turn relative position
		 */
		public Rotation2d getTurnRelativePosition() {
			return this.turnRelativePosition;
		}

		/**
		 * Set the turn relative position.
		 * @param turnRelPos
		 */
		public void setTurnRelativePosition(Rotation2d turnRelPos) {
			this.turnRelativePosition = turnRelPos;
		}

		/**
		 * Get the turn velocity.
		 * @return turn velocity
		 */
		public double getTurnVelocity() {
			return this.turnVelocity;
		}

		/**
		 * Set turn velocity.
		 * @param turnVel
		 */
		public void setTurnVelocity(double turnVel) {
			this.turnVelocity = turnVel;
		}

		/**
		 * Get turn applied volts.
		 * @return Turn applied volts
		 */
		public double getTurnAppliedVolts() {
			return this.turnAppliedVolts;
		}

		/**
		 * Set turn applied volts.
		 * @param turnVolts
		 */
		public void setTurnAppliedVolts(double turnVolts) {
			this.turnAppliedVolts = turnVolts;
		}

		/**
		 * Get turn current amps.
		 * @return Turn current amps
		 */
		public double getTurnCurrentAmps() {
			return this.turnCurrentAmps;
		}

		/**
		 * Set turn current amps.
		 * @param turnCurrent
		 */
		public void setTurnCurrentAmps(double turnCurrent) {
			this.turnCurrentAmps = turnCurrent;
		}
	}
	//TODO: Check if this should go into the ModuleIOInfo class
	/**
	 * Apply PID values to the motor controller using the motor's PID controller
	 * @param P Proportional gain
	 * @param I Integral gain
	 * @param D Derivative gain
	 * @param FF Feedforward gain
	 * @param slot PID slot to apply to
	 * @param onDriveMotor Whether to apply to the drive motor or on the turning motor
	 * @throws IllegalArgumentException if the slot number is invalid (greater than 2 on TalonFX)
	 */
	default void applyPID(double P, double I, double D, double FF, int slot, boolean onDriveMotor) throws IllegalArgumentException { }

	/**
	 * Update all the values of the logged values of the module through a ModuleIOInfo object.
	 * @param moduleInfo
	 */
	default void updateInputs(ModuleIOInfo moduleInfo) { }

	/**
	 * Run the drive motor at the specified voltage.
	 * @param volts
	*/
	default void setDriveVoltage(double volts) { }

	/**
	 * Run the turn motor at the specified voltage.
	 * @param volts
	 */
	default void setTurnVoltage(double volts) { }

	/**
	 * Set whether the drive motor is on brake mode or not.
	 * @param enable
	*/
	default void setDriveBrakeMode(boolean enable) { }

	/**
	 * Set whether the turn motor is on brake mode or not.
	 * @param enable
	 */
	default void setTurnBrakeMode(boolean enable) { }

/**
	 * Set the name of the module.
	 * @param moduleName
	 */
	default void setModuleName(String moduleName) { }

	/**
	 * Get the shorthanded logging name of the module.
	 * @return Name of the module
	 */
	default String getModuleName() {
		return "";
	}

	/** Reset the encoder position values. */
	default void resetEncoders() { }
}
