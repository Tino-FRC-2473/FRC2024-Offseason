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
		inputs.connected = true;
		inputs.yawPosition = gyroSimulation.getGyroReading();
		inputs.yawVelocity = gyroSimulation.getMeasuredAngularVelocityRadPerSec();
	}
}
