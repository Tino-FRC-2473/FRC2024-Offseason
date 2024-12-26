package frc.robot.systems.drive.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.systems.drive.gyro.GyroIO.GyroIOInfo;

public class GyroIONavX implements GyroIO {
	private AHRS gyro = new AHRS(SPI.Port.kMXP);

	/** IO implementation for AHRS NavX Gyro. */
	public GyroIONavX() {
		gyro.reset();
		gyro.setAngleAdjustment(0);
	}

	@Override
	public void updateInputs(GyroIOInfo inputs) {
		inputs.setConnected(gyro.isConnected());
		inputs.setYawPosition(Rotation2d.fromDegrees(gyro.getAngle()));
		inputs.setYawVelocity(Units.degreesToRadians(gyro.getVelocityX())); // unreferenced field
	}

	@Override
	public void resetHeading() {
		gyro.reset();
		gyro.setAngleAdjustment(0);
	}
}
