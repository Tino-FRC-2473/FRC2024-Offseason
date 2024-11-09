// package frc.robot.systems.drive;

// import edu.wpi.first.math.geometry.Rotation2d;
// import frc.robot.util.OdometryTimeStampsSim;
// import java.util.Arrays;
// import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

// /** Wrapper class around {@link SwerveModuleSimulation} that implements ModuleIO */
// public class ModuleIOSim implements ModuleIO {
//   private final SwerveModuleSimulation moduleSimulation;

//   public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
//     this.moduleSimulation = moduleSimulation;
//   }

//   @Override
//   public void updateInputs(ModuleIOInputs inputs) {
//     inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPositionRad();
//     inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeedRadPerSec();
//     inputs.driveAppliedVolts = moduleSimulation.getDriveMotorAppliedVolts();
//     inputs.driveCurrentAmps =
//         new double[] {Math.abs(moduleSimulation.getDriveMotorSupplyCurrentAmps())};

//     inputs.turnAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();
//     inputs.turnPosition =
//         Rotation2d.fromRadians(moduleSimulation.getSteerRelativeEncoderPositionRad());
//     inputs.turnVelocityRadPerSec = moduleSimulation.getSteerRelativeEncoderSpeedRadPerSec();
//     inputs.turnAppliedVolts = moduleSimulation.getSteerMotorAppliedVolts();
//     inputs.turnCurrentAmps =
//         new double[] {Math.abs(moduleSimulation.getSteerMotorSupplyCurrentAmps())};

//     inputs.odometryTimestamps = OdometryTimeStampsSim.getTimeStamps();
//     inputs.odometryDrivePositionsRad = moduleSimulation.getCachedDriveWheelFinalPositionsRad();
//     inputs.odometryTurnPositions =
//         Arrays.stream(moduleSimulation.getCachedSteerRelativeEncoderPositions())
//             .mapToObj(Rotation2d::fromRadians)
//             .toArray(Rotation2d[]::new);
//   }

//   @Override
//   public void setDriveVoltage(double volts) {
//     moduleSimulation.requestDriveVoltageOut(volts);
//   }

//   @Override
//   public void setTurnVoltage(double volts) {
//     moduleSimulation.requestSteerVoltageOut(volts);
//   }
// }