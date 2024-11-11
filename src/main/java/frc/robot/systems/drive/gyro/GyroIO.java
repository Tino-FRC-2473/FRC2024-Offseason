package frc.robot.systems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {

	class GyroIOInfo {
		private boolean connected = false;
		private Rotation2d yawPosition = new Rotation2d();
		private double yawVelocity = 0.0;

		/**
		 * Set whether or not the gyro is connected.
		 * @param isConnected
		 */
		public void setConnected(boolean isConnected) {
			connected = isConnected;
		}

		/**
		 * Get whether or not the gyro is connected.
		 * @return isConnected
		 */
		public boolean isConnected() {
			return connected;
		}

		/**
		 * Set the yaw position.
		 * @param yaw
		 */
		public void setYawPosition(Rotation2d yaw) {
			yawPosition = yaw;
		}

		/**
		 * Get the yaw position.
		 * @return yaw position
		 */
		public Rotation2d getYawPosition() {
			return yawPosition;
		}

		/**
		 * Set the yaw velocity.
		 * @param yawVel
		 */
		public void setYawVelocity(double yawVel) {
			yawVelocity = yawVel;
		}

		/**
		 * Get the yaw velocity.
		 * @return yaw velocity
		 */
		public double getYawVelocity() {
			return yawVelocity;
		}
	}

	/**
	 * Update a set of loggable inputs.
	 * @param inputs the logged gyro info
	 */
	default void updateInputs(GyroIOInfo inputs) { }

	/** Reset the heading of the gyro. */
	default void resetHeading() { }
}
