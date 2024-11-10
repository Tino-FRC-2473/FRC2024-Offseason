package frc.robot.systems.drive.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

import frc.robot.SwerveConstants;
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
  
	public ModuleIOTalonFX(int driveMotorId, int turningMotorId, int encoderId, double angleEncoderOffset) {
		driveMotor = new TalonFX(driveMotorId);
		turnMotor = new TalonFX(turningMotorId);
		turningEncoder = new CANcoder(encoderId);
		this.angleEncoderOffset = new Rotation2d(angleEncoderOffset);

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
	}

	@Override
	public void updateInputs(ModuleIOInfo inputs) {
		inputs.connected = BaseStatusSignal.refreshAll(
			drivePosition,
			driveVelocity,
			driveAppliedVolts,
			driveCurrent,
			turnAbsolutePosition,
			turnRelativePosition,
			turnVelocity,
			turnAppliedVolts,
			turnCurrent
		).equals(StatusCode.OK);

		inputs.drivePosition = drivePosition.getValueAsDouble() * 2 * Math.PI / ModuleConstants.DRIVE_GEAR_RATIO;
		inputs.driveVelocity = driveVelocity.getValueAsDouble() * 2 * Math.PI / ModuleConstants.DRIVE_GEAR_RATIO;
		inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
		inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

		inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
			.minus(angleEncoderOffset);
		inputs.turnRelativePosition = Rotation2d.fromRotations(turnRelativePosition.getValueAsDouble() / ModuleConstants.TURN_GEAR_RATIO);
		inputs.turnVelocity = turnVelocity.getValueAsDouble() * 2 * Math.PI / ModuleConstants.TURN_GEAR_RATIO;
		inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
		inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();
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
}