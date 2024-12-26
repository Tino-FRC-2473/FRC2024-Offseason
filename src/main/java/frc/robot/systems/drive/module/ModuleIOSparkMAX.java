package frc.robot.systems.drive.module;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.SwerveConstants.ModuleConstants;

public class ModuleIOSparkMAX implements ModuleIO {
	private final CANSparkMax driveMotor;
	private final CANSparkMax turnMotor;
	private final AbsoluteEncoder turningAbsEncoder;
	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder turningRelEncoder;

	private final Rotation2d angleEncoderOffset;

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
		turningRelEncoder = turnMotor.getEncoder();
		driveEncoder = driveMotor.getEncoder();
		this.angleEncoderOffset = new Rotation2d(angleOffset);

		driveEncoder.setPositionConversionFactor(
				ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);
		driveEncoder.setVelocityConversionFactor(
				ModuleConstants.DRIVING_ENCODOR_VELOCITY_FACTOR);

		turningAbsEncoder.setPositionConversionFactor(
				ModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
		turningAbsEncoder.setVelocityConversionFactor(
				ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

		driveMotor.setSmartCurrentLimit(ModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT);
		turnMotor.setSmartCurrentLimit(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);

		driveMotor.burnFlash();
		turnMotor.burnFlash();

		setModuleName(moduleName);
	}

	@Override
	public void updateInputs(ModuleIOInfo inputs) {
		inputs.setConnected(true); //find a better way to set connected

		inputs.setDrivePosition(driveEncoder.getPosition());
		inputs.setDriveVelocity(driveEncoder.getVelocity());
		inputs.setDriveAppliedVolts(turnMotor.getBusVoltage());
		inputs.setDriveCurrentAmps(driveMotor.getOutputCurrent());

		inputs.setTurnAbsolutePosition(
			Rotation2d.fromRotations(turningAbsEncoder.getPosition())
			.minus(angleEncoderOffset));
		inputs.setTurnRelativePosition(
			Rotation2d.fromRotations(turningRelEncoder.getPosition()
				/ ModuleConstants.TURN_GEAR_RATIO));
		inputs.setTurnVelocity(
			turningRelEncoder.getVelocity() / ModuleConstants.TURN_GEAR_RATIO);
		inputs.setTurnAppliedVolts(turnMotor.getBusVoltage());
		inputs.setTurnCurrentAmps(turnMotor.getOutputCurrent());
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
	public void applyPID(double P, double I, double D, double FF, int slot, boolean onDriveMotor) {
		//TODO: Is the pass by value done by java here mean that we get the reference to each pid controller?
		SparkPIDController pidController = onDriveMotor ? driveMotor.getPIDController() : turnMotor.getPIDController();
		pidController.setP(P, slot);
		pidController.setI(I, slot);
		pidController.setD(D, slot);
		pidController.setFF(FF, slot);
	}
}
