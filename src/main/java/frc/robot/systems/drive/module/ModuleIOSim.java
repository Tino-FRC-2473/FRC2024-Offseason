package frc.robot.systems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/** Wrapper class around {@link SwerveModuleSimulation} that implements ModuleIO */
public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation moduleSimulation;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
  }

  @Override
  public void updateInputs(ModuleIOInfo inputs) {
    inputs.drivePosition = moduleSimulation.getDriveWheelFinalPositionRad();
    inputs.driveVelocity = moduleSimulation.getDriveWheelFinalSpeedRadPerSec();
    inputs.driveAppliedVolts = moduleSimulation.getDriveMotorAppliedVolts();
    inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorSupplyCurrentAmps());

    inputs.turnAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnRelativePosition = Rotation2d.fromRadians(moduleSimulation.getSteerRelativeEncoderPositionRad());
    inputs.turnVelocity = moduleSimulation.getSteerRelativeEncoderSpeedRadPerSec();
    inputs.turnAppliedVolts = moduleSimulation.getSteerMotorAppliedVolts();
    inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorSupplyCurrentAmps());
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