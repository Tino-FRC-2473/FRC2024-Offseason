package frc.robot.systems.drive.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.HardwareMap;

public class GyroIOPigeon2 implements GyroIO {
	private final Pigeon2 pigeon = new Pigeon2(HardwareMap.GYRO_ID);
	private final StatusSignal<Double> yaw = pigeon.getYaw();
	private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZWorld();

	/** IO implementation for Pigeon2. */
	public GyroIOPigeon2() {
		pigeon.getConfigurator().apply(new Pigeon2Configuration());
		pigeon.getConfigurator().setYaw(0.0);
		pigeon.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(GyroIOInfo inputs) {
		inputs.setConnected(BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK));
		inputs.setYawPosition(Rotation2d.fromDegrees(yaw.getValueAsDouble()));
		inputs.setYawVelocity(Units.degreesToRadians(yawVelocity.getValueAsDouble()));
	}

	@Override
	public void resetHeading() {
		pigeon.reset();
		pigeon.setYaw(0);
	}
}
