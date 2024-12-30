package frc.robot.systems.drive.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.SwerveConstants.ModuleConstants;

public class ModuleIOTalonFX implements ModuleIO {
	private final TalonFX driveMotor;
	private final TalonFX turnMotor;
	private final CANcoder turningEncoder;

	private final boolean isTurnMotorInverted = true;
	private final Rotation2d angleEncoderOffset;

	//Status Signals --> NOT JUST FOR LOGGING
	private final StatusSignal<Double> drivePosition;
	private final StatusSignal<Double> driveVelocity;
	private final StatusSignal<Double> driveAppliedVolts;
	private final StatusSignal<Double> driveCurrent;

	private final StatusSignal<Double> turnAbsolutePosition;
	private final StatusSignal<Double> turnRelativePosition;
	private final StatusSignal<Double> turnVelocity;
	private final StatusSignal<Double> turnAppliedVolts;
	private final StatusSignal<Double> turnCurrent;

	private String mName;

	/**
	 * Makes a ModuleIOTalonFX object that models a MK4n with 2 Krakenx60s,
	 * a CANCoder, and specified angle offset.
	 *
	 * @param driveMotorId
	 * @param turningMotorId
	 * @param encoderId
	 * @param angleOffset
	 * @param moduleName
	 */
	public ModuleIOTalonFX(int driveMotorId, int turningMotorId,
		int encoderId, double angleOffset, String moduleName) {
		driveMotor = new TalonFX(driveMotorId);
		turnMotor = new TalonFX(turningMotorId);
		turningEncoder = new CANcoder(encoderId);
		this.angleEncoderOffset = new Rotation2d(angleOffset);

		TalonFXConfiguration driveConfig = new TalonFXConfiguration();
		driveConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT;
		driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

		TalonFXConfiguration turnConfig = new TalonFXConfiguration();
		turnConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT;
		turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

		CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

		driveMotor.getConfigurator().apply(driveConfig);
		turnMotor.getConfigurator().apply(turnConfig);
		turningEncoder.getConfigurator().apply(encoderConfig);

		drivePosition = driveMotor.getPosition();
		driveVelocity = driveMotor.getVelocity();
		driveAppliedVolts = driveMotor.getMotorVoltage();
		driveCurrent = driveMotor.getSupplyCurrent();

		turnAbsolutePosition = turningEncoder.getAbsolutePosition();
		turnRelativePosition = turnMotor.getPosition();
		turnVelocity = turnMotor.getVelocity();
		turnAppliedVolts = turnMotor.getMotorVoltage();
		turnCurrent = turnMotor.getSupplyCurrent();

		driveMotor.optimizeBusUtilization();
		turnMotor.optimizeBusUtilization();

		mName = moduleName;
	}

	@Override
	public void updateInputs(ModuleIOInfo inputs) {
		inputs.setConnected(BaseStatusSignal.refreshAll(
			drivePosition,
			driveVelocity,
			driveAppliedVolts,
			driveCurrent,
			turnAbsolutePosition,
			turnRelativePosition,
			turnVelocity,
			turnAppliedVolts,
			turnCurrent
		).equals(StatusCode.OK));

		inputs.setDrivePosition(
			drivePosition.getValueAsDouble() * 2 * Math.PI / ModuleConstants.DRIVE_GEAR_RATIO);
		inputs.setDriveVelocity(
			driveVelocity.getValueAsDouble() * 2 * Math.PI / ModuleConstants.DRIVE_GEAR_RATIO);
		inputs.setDriveAppliedVolts(driveAppliedVolts.getValueAsDouble());
		inputs.setDriveCurrentAmps(driveCurrent.getValueAsDouble());

		inputs.setTurnAbsolutePosition(
			Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
			.minus(angleEncoderOffset));
		inputs.setTurnRelativePosition(
			Rotation2d.fromRotations(turnRelativePosition.getValueAsDouble()
				/ ModuleConstants.TURN_GEAR_RATIO));
		inputs.setTurnVelocity(
			turnVelocity.getValueAsDouble() * 2 * Math.PI / ModuleConstants.TURN_GEAR_RATIO);
		inputs.setTurnAppliedVolts(turnAppliedVolts.getValueAsDouble());
		inputs.setTurnCurrentAmps(turnCurrent.getValueAsDouble());
	}

	@Override
	public void setDriveVoltage(double volts) {
		driveMotor.setControl(new VoltageOut(volts));
	}

	@Override
	public void setTurnVoltage(double volts) {
		turnMotor.setControl(new VoltageOut(volts));
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		var config = new MotorOutputConfigs();
		config.Inverted = InvertedValue.CounterClockwise_Positive;
		config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		driveMotor.getConfigurator().apply(config);
	}

	@Override
	public void setTurnBrakeMode(boolean enable) {
		var config = new MotorOutputConfigs();
		config.Inverted =
			isTurnMotorInverted
				? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		turnMotor.getConfigurator().apply(config);
	}

	@Override
	public void resetEncoders() {
		driveMotor.setPosition(0);
	}

	@Override
	public void setModuleName(String moduleName) {
		mName = moduleName;
	}

	@Override
	public String getModuleName() {
		return mName;
	}
}
