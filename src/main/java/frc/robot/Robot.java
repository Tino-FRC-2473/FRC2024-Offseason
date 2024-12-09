// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.Orchestra;
// Third Party Imports
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;
	private TalonFX motor;
	private Orchestra orchestra;
	// Systems

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();
		motor = new TalonFX(HardwareMap.MOTOR_ID);
		motor.setNeutralMode(NeutralModeValue.Brake);

		orchestra = new Orchestra();
		orchestra.addInstrument(motor);
		orchestra.loadMusic("f1.chrp");
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
		motor.set(0);
		orchestra.stop();
	}

	@Override
	public void teleopPeriodic() {
		motor.set(0);

		if (input.isPlayButtonPressed() && !orchestra.isPlaying()) {
			orchestra.play();
		} else if (input.isPauseButtonPressed() && orchestra.isPlaying()) {
			orchestra.pause();
		} else if (input.isStopButtonPressed()) {
			orchestra.stop();
		}
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
		SmartDashboard.putBoolean("PLAY BUTTON", input.isPlayButtonPressed());
		SmartDashboard.putBoolean("PAUSE BUTTON", input.isPauseButtonPressed());
		SmartDashboard.putBoolean("STOP BUTTON", input.isStopButtonPressed());
	}
}
