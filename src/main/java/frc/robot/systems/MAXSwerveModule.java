// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.SwerveConstants.ModuleConstants;

public class MAXSwerveModule {
	private final CANSparkMax drivingSparkMax;
	private final CANSparkMax turningSparkMax;

	private final RelativeEncoder drivingEncoder;
	private final AbsoluteEncoder turningEncoder;

	private final SparkPIDController drivingPIDController;
	private final SparkPIDController turningPIDController;

	private double chassisAngularOffset = 0;
	private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

	/**
	 * Constructs a MAXSwerveModule and configures the driving and turning motor,
	 * encoder, and PID controller. This configuration is specific to the REV
	 * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
	 * Encoder.
	 *
	 * @param drivingCANId id of the driving motor
	 * @param turningCANId id of the turning motor
	 * @param newChassisAngularOffset offset of the angle due to the chassis' rotation
	 */
	public MAXSwerveModule(int drivingCANId, int turningCANId, double newChassisAngularOffset) {
		drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
		turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

		// Setup encoders and PID controllers for the driving and turning SPARKS MAX.
		drivingEncoder = drivingSparkMax.getEncoder();
		turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
		drivingPIDController = drivingSparkMax.getPIDController();
		turningPIDController = turningSparkMax.getPIDController();
		drivingPIDController.setFeedbackDevice(drivingEncoder);
		turningPIDController.setFeedbackDevice(turningEncoder);

		// Apply position and velocity conversion factors for the driving encoder. The
		// native units for position and velocity are rotations and RPM, respectively,
		// but we want meters and meters per second to use with WPILib's swerve APIs.
		drivingEncoder.setPositionConversionFactor(ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);
		drivingEncoder.setVelocityConversionFactor(ModuleConstants.DRIVING_ENCODOR_VELOCITY_FACTOR);

		// Apply position and velocity conversion factors for the turning encoder. We
		// want these in radians and radians per second to use with WPILib's swerve
		// APIs.
		turningEncoder.setPositionConversionFactor(ModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
		turningEncoder.setVelocityConversionFactor(ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

		// Invert the turning encoder, since the output shaft rotates in the opposite direction of
		// the steering motor in the MAXSwerve Module.
		turningEncoder.setInverted(ModuleConstants.TURNING_MOTOR_INVERTED);

		// Enable PID wrap around for the turning motor. This will allow the PID
		// controller to go through 0 to get to the setpoint i.e. going from 350 degrees
		// to 10 degrees will go through 0 rather than the other direction which is a
		// longer route.
		turningPIDController.setPositionPIDWrappingEnabled(true);
		turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants
			.TURNING_ENCODER_POSITION_PID_MIN_INPUT);
		turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants
			.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

		// Set the PID gains for the driving motor. Note these are example gains, and you
		// may need to tune them for your own robot!
		drivingPIDController.setP(ModuleConstants.DRIVING_P);
		drivingPIDController.setI(ModuleConstants.DRIVING_I);
		drivingPIDController.setD(ModuleConstants.DRIVING_D);
		drivingPIDController.setFF(ModuleConstants.DRIVING_FF);
		drivingPIDController.setOutputRange(ModuleConstants.DRIVING_MIN_OUTPUT,
				ModuleConstants.DRIVING_MAX_OUTPUT);

		// Set the PID gains for the turning motor. Note these are example gains, and you
		// may need to tune them for your own robot!
		turningPIDController.setP(ModuleConstants.TURNING_P);
		turningPIDController.setI(ModuleConstants.TURNING_I);
		turningPIDController.setD(ModuleConstants.TURNING_D);
		turningPIDController.setFF(ModuleConstants.TURNING_FF);
		turningPIDController.setOutputRange(ModuleConstants.TURNING_MIN_OUTPUT,
				ModuleConstants.TURNING_MAX_OUTPUT);

		drivingSparkMax.setIdleMode(ModuleConstants.DRIVING_MOTOR_IDLE_MODE);
		turningSparkMax.setIdleMode(ModuleConstants.TURNING_MOTOR_IDLE_MODE);
		drivingSparkMax.setSmartCurrentLimit(ModuleConstants.DRIVING_MOTOR_CUTTENT_LIMIT);
		turningSparkMax.setSmartCurrentLimit(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);

		// Save the SPARK MAX configurations. If a SPARK MAX browns out during
		// operation, it will maintain the above configurations.
		drivingSparkMax.burnFlash();
		turningSparkMax.burnFlash();

		chassisAngularOffset = newChassisAngularOffset;
		desiredState.angle = new Rotation2d(turningEncoder.getPosition());
		drivingEncoder.setPosition(0);
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		// Apply chassis angular offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModuleState(drivingEncoder.getVelocity(),
				new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
	}

	/**
	 * Returns the current position of the module.
	 *
	 * @return The current position of the module.
	 */
	public SwerveModulePosition getPosition() {
		// Apply chassis angular offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModulePosition(
				drivingEncoder.getPosition(),
				new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
	}

	/**
	 * Sets the desired state for the module.
	 *
	 * @param newDesiredState Desired state with speed and angle.
	 */
	public void setDesiredState(SwerveModuleState newDesiredState) {
		// Apply chassis angular offset to the desired state.
		SwerveModuleState correctedDesiredState = new SwerveModuleState();
		correctedDesiredState.speedMetersPerSecond = newDesiredState.speedMetersPerSecond;
		correctedDesiredState.angle = newDesiredState.angle
			.plus(Rotation2d.fromRadians(chassisAngularOffset));

		// Optimize the reference state to avoid spinning further than 90 degrees.
		SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
				new Rotation2d(turningEncoder.getPosition()));

		SmartDashboard.putNumber("Driving Spark MAX" + drivingSparkMax.getDeviceId(),
			optimizedDesiredState.speedMetersPerSecond);
		SmartDashboard.putNumber("Turning Spark MAX" + turningSparkMax.getDeviceId(),
			optimizedDesiredState.angle.getRadians());

		// Command driving and turning SPARKS MAX towards their respective setpoints.
		drivingPIDController.setReference(optimizedDesiredState
			.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
		turningPIDController.setReference(optimizedDesiredState.angle.getRadians(),
			CANSparkMax.ControlType.kPosition);

		desiredState = newDesiredState;
	}

	/** Zeroes all the SwerveModule encoders. */
	public void resetEncoders() {
		drivingEncoder.setPosition(0);
	}
}
