package frc.robot.systems.drive.module;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.SwerveConstants.ModuleConstants;

public class ModuleIOSparkMAX implements ModuleIO {
	private final CANSparkMax driveMotor;
	private final CANSparkMax turnMotor;
	private final AbsoluteEncoder turningAbsEncoder;
	private final RelativeEncoder driveEncoder;
	private final SparkPIDController driveController;
	private final SparkPIDController turnController;

	private final Rotation2d angleEncoderOffset;

	private String mName;

	/**
	 * Makes a ModuleIOTalonFX object that models a MK4n with 2 Krakenx60s,
	 * a CANCoder, and specified angle offset.
	 *
	 * @param driveMotorId
	 * @param turningMotorId
	 * @param angleOffset
	 * @param moduleName
	 */
	public ModuleIOSparkMAX(int driveMotorId, int turningMotorId,
		double angleOffset, String moduleName) {
		driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
		turnMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
		turningAbsEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
		driveEncoder = driveMotor.getEncoder();
		this.angleEncoderOffset = new Rotation2d(angleOffset);

		driveController = driveMotor.getPIDController();
		turnController = turnMotor.getPIDController();

		driveMotor.setSmartCurrentLimit(ModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT);
		turnMotor.setSmartCurrentLimit(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);

		driveEncoder.setPositionConversionFactor(
			ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);
		driveEncoder.setVelocityConversionFactor(
				ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

		turningAbsEncoder.setPositionConversionFactor(
			ModuleConstants.TURNING_ENCODER_POSITION_FACTOR
		);

		turningAbsEncoder.setVelocityConversionFactor(
			ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR
		);

		driveController.setFeedbackDevice(driveEncoder);
		turnController.setFeedbackDevice(turningAbsEncoder);

		//Turn controller cont input from 0 -> 2pi
		turnController.setPositionPIDWrappingEnabled(true);
		turnController.setPositionPIDWrappingMinInput(
			ModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT
		);
		turnController.setPositionPIDWrappingMaxInput(
			ModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT
		);

		driveController.setP(ModuleConstants.DRIVING_P);
		driveController.setI(ModuleConstants.DRIVING_I);
		driveController.setD(ModuleConstants.DRIVING_D);
		driveController.setFF(ModuleConstants.DRIVING_FF);
		driveController.setOutputRange(ModuleConstants.DRIVING_MIN_OUTPUT,
				ModuleConstants.DRIVING_MAX_OUTPUT);

		turnController.setP(ModuleConstants.TURNING_P);
		turnController.setI(ModuleConstants.TURNING_I);
		turnController.setD(ModuleConstants.TURNING_D);
		turnController.setFF(ModuleConstants.TURNING_FF);
		turnController.setOutputRange(ModuleConstants.TURNING_MIN_OUTPUT,
				ModuleConstants.TURNING_MAX_OUTPUT);

		setDriveBrakeMode(true);
		setTurnBrakeMode(true);

		driveMotor.burnFlash();
		turnMotor.burnFlash();

		mName = moduleName;
	}

	@Override
	public void updateInputs(ModuleIOInfo inputs) {
		inputs.setConnected(
			driveMotor.getLastError() == REVLibError.kOk
			&& turnMotor.getLastError() == REVLibError.kOk
		); //find a better way to set connected

		if (inputs.isConnected()) {
			inputs.setDrivePosition(driveEncoder.getPosition());
			inputs.setDriveVelocity(driveEncoder.getVelocity());
			inputs.setDriveAppliedVolts(turnMotor.getBusVoltage());
			inputs.setDriveCurrentAmps(driveMotor.getOutputCurrent());

			inputs.setTurnAbsolutePosition(
				Rotation2d.fromRotations(turningAbsEncoder.getPosition())
				.minus(angleEncoderOffset));
			inputs.setTurnRelativePosition(
				Rotation2d.fromRotations(0)); // pretty sure this is a redundant measure.
			inputs.setTurnVelocity(turningAbsEncoder.getVelocity());
			inputs.setTurnAppliedVolts(turnMotor.getBusVoltage() * turnMotor.getAppliedOutput());
			inputs.setTurnCurrentAmps(turnMotor.getOutputCurrent());
		} else {
			inputs.setDrivePosition(0);
			inputs.setDriveVelocity(0);
			inputs.setDriveAppliedVolts(0);
			inputs.setDriveCurrentAmps(0);

			inputs.setTurnAbsolutePosition(
				Rotation2d.fromRotations(0));
			inputs.setTurnRelativePosition(
				Rotation2d.fromRotations(0)); // pretty sure this is a redundant measure.
			inputs.setTurnVelocity(0);
			inputs.setTurnAppliedVolts(0);
			inputs.setTurnCurrentAmps(0);
		}
	}

	@Override
	public void setDriveVoltage(double volts) {
		driveMotor.setVoltage(volts);
	}

	@Override
	public void setTurnVoltage(double volts) {
		turnMotor.setVoltage(volts);
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		driveMotor.setInverted(ModuleConstants.DRIVING_MOTOR_INVERTED);
		driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
	}

	@Override
	public void setTurnBrakeMode(boolean enable) {
		turnMotor.setInverted(ModuleConstants.TURNING_MOTOR_INVERTED);
		turnMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
	}

	@Override
	public void resetEncoders() {
		driveEncoder.setPosition(0);
	}

	@Override
	public String getModuleName() {
		return mName;
	}

	@Override
	public void setModuleName(String moduleName) {
		mName = moduleName;
	}

	@Override
	public void setDriveVelocity(double velocity) {
		double ffVolts = ModuleConstants.DRIVING_FF_KS * Math.signum(velocity)
			+ ModuleConstants.DRIVING_FF_KV * velocity;
		driveController.setReference(velocity, ControlType.kVelocity,
			0, ffVolts, ArbFFUnits.kVoltage);
		//driveController.setReference(velocity, ControlType.kVelocity);
	}

	@Override
	public void setTurnPosition(Rotation2d rotation) {
		double setpoint =
			MathUtil.inputModulus(rotation.plus(angleEncoderOffset).getRadians(),
				ModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT,
				ModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);
		turnController.setReference(setpoint, ControlType.kPosition);
	}
}
