// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third Party Imports


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;
	// Systems
	private AnalogInput sharp;
	private DigitalInput breakbeam;
	private TimeOfFlight pwFlight;
	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();
		sharp = new AnalogInput(0);
		breakbeam = new DigitalInput(0);
		pwFlight = new TimeOfFlight(0);
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
	}

	@Override
	public void teleopPeriodic() {
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
	public void simulationPeriodic() { }

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() {
		SmartDashboard.putBoolean("beam break", breakbeam.get());
		SmartDashboard.putNumber("sharp ToF", sharp.getValue());
		SmartDashboard.putNumber("PWF Distance mm", pwFlight.getRange());
	}
}
