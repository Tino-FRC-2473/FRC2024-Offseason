// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;

// Third Party Imports
import com.ctre.phoenix6.SignalLogger;

import frc.robot.systems.BreakBeamSensorFSM;
// Systems
import frc.robot.systems.DeployerFSM;
import frc.robot.systems.DistanceSensorFSM;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;
	// Systems
	private DeployerFSM deployerFSM;
	private BreakBeamSensorFSM breakFSM;
	private DistanceSensorFSM dsFSM;
	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();

		// Instantiate all systems here
		deployerFSM = new DeployerFSM();
		breakFSM = new BreakBeamSensorFSM();
		dsFSM = new DistanceSensorFSM(); 
	}


	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
	}


	@Override
	public void autonomousPeriodic() {	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		deployerFSM.reset();
		SignalLogger.start();
	}

	@Override
	public void teleopPeriodic() {
		deployerFSM.update(input);
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
		SignalLogger.stop();
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
	public void simulationPeriodic() { }

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() { }
}
