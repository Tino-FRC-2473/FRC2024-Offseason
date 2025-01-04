

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.SwerveConstants.ModuleConstants;
import frc.robot.util.SparkUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
	private final SwerveModuleSimulation moduleSimulation;
	private final DCMotorSim driveMotor;
	private final DCMotorSim turnMotor;

	private boolean driveClosedLoop = false;
	private boolean turnClosedLoop = false;
	private final PIDController driveController =
		new PIDController(ModuleConstants.DRIVING_P, 0, ModuleConstants.DRIVING_D);
	private final PIDController turnController =
		new PIDController(ModuleConstants.TURNING_P, 0, ModuleConstants.TURNING_D);
	private double driveFFVolts = 0.0;
	private double driveAppliedVolts = 0.0;
	private double turnAppliedVolts = 0.0;

	public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
		this.moduleSimulation = moduleSimulation;
		this.driveMotor =
				moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(
					Amps.of(ModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT));
		this.turnMotor =
				moduleSimulation.useGenericControllerForSteer().withCurrentLimit(
					Amps.of(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT));

		// Enable wrapping for turn PID
		turnController.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void updateInputs(ModuleIOInfo inputs) {
		// Run closed-loop control
		if (driveClosedLoop) {
			driveAppliedVolts = driveFFVolts
					+ driveController.calculate(
							moduleSimulation.getDriveWheelFinalSpeedRadPerSec());
		} else {
			driveController.reset();
		}
		if (turnClosedLoop) {
			turnAppliedVolts = turnController.calculate(
					moduleSimulation.getSteerAbsoluteFacing().getRadians());
		} else {
			turnController.reset();
		}

		// Update simulation state
		driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
		turnMotor.requestVoltage(Volts.of(turnAppliedVolts));

		// Update drive inputs
		inputs.driveConnected = true;
		inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
		inputs.driveVelocityRadPerSec =
				moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
		inputs.driveAppliedVolts = driveAppliedVolts;
		inputs.driveCurrentAmps =
				Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));

		// Update turn inputs
		inputs.turnConnected = true;
		inputs.turnPosition = moduleSimulation.getSteerAbsoluteFacing();
		inputs.turnVelocityRadPerSec =
				moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
		inputs.turnAppliedVolts = turnAppliedVolts;
		inputs.turnCurrentAmps =
				Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));

		// Update odometry inputs
		inputs.odometryTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
		inputs.odometryDrivePositionsRad = Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
				.mapToDouble(angle -> angle.in(Radians))
				.toArray();
		inputs.odometryTurnPositions = moduleSimulation.getCachedSteerAbsolutePositions();
	}

	@Override
	public void setDriveOpenLoop(double output) {
		driveClosedLoop = false;
		driveAppliedVolts = output;
	}

	@Override
	public void setTurnOpenLoop(double output) {
		turnClosedLoop = false;
		turnAppliedVolts = output;
	}

	@Override
	public void setDriveVelocity(double velocityRadPerSec) {
		driveClosedLoop = true;
		driveFFVolts = driveSimKs * Math.signum(velocityRadPerSec) + driveSimKv * velocityRadPerSec;
		driveController.setSetpoint(velocityRadPerSec);
	}

	@Override
	public void setTurnPosition(Rotation2d rotation) {
		turnClosedLoop = true;
		turnController.setSetpoint(rotation.getRadians());
	}
}