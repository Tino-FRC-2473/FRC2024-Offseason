package frc.robot.systems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/** Wrapper class around {@link SwerveModuleSimulation} that implements ModuleIO. */
public class ModuleIOSim implements ModuleIO {
	private final SwerveModuleSimulation moduleSimulation;

	/**
	 * Creates a ModuleIOSim object that takes a MapleSim SwerveModSim object.
	 * @param moduleSim
	 */
	public ModuleIOSim(SwerveModuleSimulation moduleSim) {
		moduleSimulation = moduleSim;
	}

	@Override
	public void updateInputs(ModuleIOInfo inputs) {
		inputs.setConnected(true);
		inputs.setDrivePosition(moduleSimulation.getDriveWheelFinalPositionRad());
		inputs.setDriveVelocity(moduleSimulation.getDriveWheelFinalSpeedRadPerSec());
		inputs.setDriveAppliedVolts(moduleSimulation.getDriveMotorAppliedVolts());
		inputs.setDriveCurrentAmps(Math.abs(moduleSimulation.getDriveMotorSupplyCurrentAmps()));

		inputs.setTurnAbsolutePosition(moduleSimulation.getSteerAbsoluteFacing());
		inputs.setTurnRelativePosition(Rotation2d.fromRadians(
				moduleSimulation.getSteerRelativeEncoderPositionRad()));
		inputs.setTurnVelocity(moduleSimulation.getSteerRelativeEncoderSpeedRadPerSec());
		inputs.setTurnAppliedVolts(moduleSimulation.getSteerMotorAppliedVolts());
		inputs.setTurnCurrentAmps(Math.abs(moduleSimulation.getSteerMotorSupplyCurrentAmps()));
	}

	@Override
	public void setDriveVoltage(double volts) {
		moduleSimulation.requestDriveVoltageOut(volts);
	}

	@Override
	public void setTurnVoltage(double volts) {
		moduleSimulation.requestSteerVoltageOut(volts);
	}
}
