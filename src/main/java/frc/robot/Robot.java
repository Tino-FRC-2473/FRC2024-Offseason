// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Third Party Imports
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;
	private TalonFX motor;
	private MotionMagicVelocityVoltage mRequest;
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
		motor.setNeutralMode(NeutralModeValue.Coast);

		var talonConfigs = new TalonFXConfiguration();
		var slot0Configs = talonConfigs.Slot0;
		var mmConfigs = talonConfigs.MotionMagic;

		slot0Configs.kS = Constants.SLOT_0_S;
		slot0Configs.kV = Constants.SLOT_0_V;
		slot0Configs.kA = Constants.SLOT_0_A;
		slot0Configs.kP = Constants.SLOT_0_P;
		slot0Configs.kI = 0;
		slot0Configs.kD = 0;

		mmConfigs.MotionMagicAcceleration = Constants.MMAGIC_CONSTANT_A;
		mmConfigs.MotionMagicJerk = Constants.MMAGIC_CONSTANT_J;

		mRequest = new MotionMagicVelocityVoltage(Constants.TARGET_VELO_RPS);

		motor.getConfigurator().apply(talonConfigs);
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
	}

	@Override
	public void teleopPeriodic() {
		if (input.isShooterButtonPressed()) {
			motor.setControl(mRequest);
		} else {
			motor.set(0);
		}
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
	public void robotPeriodic() {
		SmartDashboard.putNumber("Velocity", motor.getVelocity().getValueAsDouble());
		SmartDashboard.putNumber("Acceleration", motor.getAcceleration().getValueAsDouble());
		SmartDashboard.putNumber("Voltage", motor.getMotorVoltage().getValueAsDouble());
	}
}
