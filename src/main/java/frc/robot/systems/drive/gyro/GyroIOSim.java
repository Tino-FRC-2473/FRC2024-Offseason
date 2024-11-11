package frc.robot.systems.drive.gyro;

import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
	private final GyroSimulation gyroSimulation;

	/**
	 * Creating a GyroIOSim based on a MapleSim GyroSimulation object.
	 * @param gyroSim
	 */
	public GyroIOSim(GyroSimulation gyroSim) {
		this.gyroSimulation = gyroSim;
	}

	@Override
	public void updateInputs(GyroIOInfo inputs) {
		inputs.setConnected(true);
		inputs.setYawPosition(gyroSimulation.getGyroReading());
		inputs.setYawVelocity(gyroSimulation.getMeasuredAngularVelocityRadPerSec());
	}
}
