// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

// Third Party Imports
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation.DRIVE_WHEEL_TYPE;
import org.littletonrobotics.junction.LogFileUtil;

import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.Constants.MatchConstants;
import frc.robot.SwerveConstants.DriveConstants;
// Systems
import frc.robot.systems.drive.DriveFSMSystem;
import frc.robot.systems.drive.gyro.GyroIO;
// IO Implementations
import frc.robot.systems.drive.gyro.GyroIOPigeon2;
import frc.robot.systems.drive.gyro.GyroIOSim;
import frc.robot.systems.drive.module.ModuleIO;
import frc.robot.systems.drive.module.ModuleIOSim;
import frc.robot.systems.drive.module.ModuleIOTalonFX;

import java.util.List;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends LoggedRobot {
	private TeleopInput input;

	// Systems
	private DriveFSMSystem driveFSMSystem;
	private SwerveDriveSimulation swerveDriveSimulation;

	private LoggedDashboardChooser<Command> autoChooser;
	private Command autonomousCommand;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();

		switch (MatchConstants.CURRENT_MODE) {
			case REAL:
				this.swerveDriveSimulation = null;

				driveFSMSystem = new DriveFSMSystem(
					new GyroIOPigeon2(),
					new ModuleIOTalonFX(// front left
						HardwareMap.FRONT_LEFT_DRIVING_CAN_ID,
						HardwareMap.FRONT_LEFT_TURNING_CAN_ID,
						HardwareMap.FRONT_LEFT_CANCODER_ID,
						DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET
					),
					new ModuleIOTalonFX(// front right
						HardwareMap.FRONT_RIGHT_DRIVING_CAN_ID,
						HardwareMap.FRONT_RIGHT_TURNING_CAN_ID,
						HardwareMap.FRONT_RIGHT_CANCODER_ID,
						DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET
					),
					new ModuleIOTalonFX(// back left
						HardwareMap.REAR_LEFT_DRIVING_CAN_ID,
						HardwareMap.REAR_LEFT_TURNING_CAN_ID,
						HardwareMap.REAR_LEFT_CANCODER_ID,
						DriveConstants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET
					),
					new ModuleIOTalonFX(// back right
						HardwareMap.REAR_RIGHT_DRIVING_CAN_ID,
						HardwareMap.REAR_RIGHT_TURNING_CAN_ID,
						HardwareMap.REAR_RIGHT_CANCODER_ID,
						DriveConstants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET
					)
				);

				Logger.addDataReceiver(new WPILOGWriter());
				Logger.addDataReceiver(new NT4Publisher());

				break;
			case SIM:
				final GyroSimulation gyroSimulation = GyroSimulation.createPigeon2();

				this.swerveDriveSimulation =
					new SwerveDriveSimulation(
						DriveConstants.ROBOT_MASS,
						DriveConstants.TRACK_WIDTH,
						DriveConstants.TRACK_WIDTH,
						DriveConstants.BUMPER_WIDTH,
						DriveConstants.BUMPER_WIDTH,
						SwerveModuleSimulation.getMark4n(
							DCMotor.getKrakenX60(1),
							DCMotor.getKrakenX60(1),
							DriveConstants.CURRENT_THRESHOLD,
							DRIVE_WHEEL_TYPE.TIRE,
							(2 + 1)), //13 gear ratio
						gyroSimulation,
						new Pose2d(1.4, 5.6, new Rotation2d())
					);

				SimulatedArena.getInstance()
					.addDriveTrainSimulation(swerveDriveSimulation);

				SimulatedArena.getInstance().resetFieldForAuto();

				driveFSMSystem =
					new DriveFSMSystem(
						new GyroIOSim(gyroSimulation),
						new ModuleIOSim(swerveDriveSimulation.getModules()[0]),
						new ModuleIOSim(swerveDriveSimulation.getModules()[1]),
						new ModuleIOSim(swerveDriveSimulation.getModules()[2]),
						new ModuleIOSim(swerveDriveSimulation.getModules()[(2 + 1)]));

				//driveFSMSystem.setPose(new Pose2d(1.4, 5.6, new Rotation2d()));

				Logger.addDataReceiver(new NT4Publisher());

				break;
			default:
				/* Replayed robot, disable IO implementations */

				/* physics simulations are also not needed */
				this.swerveDriveSimulation = null;
				driveFSMSystem =
					new DriveFSMSystem(
						new GyroIO() { },
						new ModuleIO() { },
						new ModuleIO() { },
						new ModuleIO() { },
						new ModuleIO() { });

				setUseTiming(false); // Run as fast as possible
				String logPath = LogFileUtil.findReplayLog();
				Logger.setReplaySource(new WPILOGReader(logPath));
				Logger.addDataReceiver(
						new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));

				break;
		}

		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

		// Set up SysId routines
		autoChooser.addOption(
			"Drive SysId (Quasistatic Forward)",
			driveFSMSystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
			"Drive SysId (Quasistatic Reverse)",
			driveFSMSystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		autoChooser.addOption(
			"Drive SysId (Dynamic Forward)",
			driveFSMSystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
			"Drive SysId (Dynamic Reverse)",
			driveFSMSystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

		// Start AKit Logger
		Logger.start();
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		//driveFSMSystem.resetAutonomus();
		autonomousCommand = getAutonomousCommand();
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
		//driveFSMSystem.updateAutonomous();
		//mField.setRobotPose(driveFSMSystem.getPose());
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		//driveFSMSystem.reset();
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
		driveFSMSystem.update(input);
		//mField.setRobotPose(driveFSMSystem.getPose());
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
	}

	@Override
	public void disabledPeriodic() {

	}

	/* Simulation mode handlers, only used for simulation testing  */
	@Override
	public void simulationInit() {
		System.out.println("-------- Simulation Init --------");
	}

	@Override
	public void simulationPeriodic() {
		driveFSMSystem.update(input);
		updateSimulationField();
	}

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() { }

	/**
	 * Get Autonomous Path Selected.
	 * @return Returns the value selected by the auto chooser.
	 */
	public Command getAutonomousCommand() {
		return autoChooser.get();
	}

	/**
	 * Updates the Maple-Sim simulation field setup (field + sim robot pose).
	 */
	public void updateSimulationField() {
		if (swerveDriveSimulation != null) {
			SimulatedArena.getInstance().simulationPeriodic();

			Logger.recordOutput("FieldSimulation/RobotPosition",
				swerveDriveSimulation.getSimulatedDriveTrainPose());

			final List<Pose3d> notes = SimulatedArena.getInstance().getGamePiecesByType("Note");
			if (notes != null) Logger.recordOutput("FieldSimulation/Notes", notes.toArray(Pose3d[]::new));

			driveFSMSystem.setPose(swerveDriveSimulation.getSimulatedDriveTrainPose());
		}
	}
}
