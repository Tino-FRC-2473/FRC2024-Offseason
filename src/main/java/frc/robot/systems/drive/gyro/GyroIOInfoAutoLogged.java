package frc.robot.systems.drive.gyro;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroIOInfoAutoLogged extends GyroIO.GyroIOInfo implements LoggableInputs, Cloneable {
	@Override
	public void toLog(LogTable table) {
		table.put("Connected", isConnected());
		table.put("YawPosition", getYawPosition());
		table.put("YawVelocity", getYawVelocity());
	}

	@Override
	public void fromLog(LogTable table) {
		setConnected(table.get("Connected", isConnected()));
		setYawPosition(table.get("YawPosition", getYawPosition()));
		setYawVelocity(table.get("YawVelocity", getYawVelocity()));
	}

	/**
	 * Clone GyroIOInfo object.
	 * @return cloned GyroIOInfo
	 */
	public GyroIOInfoAutoLogged clone() {
		GyroIOInfoAutoLogged copy = new GyroIOInfoAutoLogged();
		copy.setConnected(this.isConnected());
		copy.setYawPosition(this.getYawPosition());
		copy.setYawVelocity(this.getYawVelocity());
		return copy;
	}
}
