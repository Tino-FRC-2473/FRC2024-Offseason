package frc.robot.systems.drive.module;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleIOInfoAutoLogged extends ModuleIO.ModuleIOInfo
	implements LoggableInputs, Cloneable {

	@Override
	public void toLog(LogTable table) {
		table.put("Connected", isConnected());
		table.put("DrivePosition", getDrivePosition());
		table.put("DriveVelocity", getDriveVelocity());
		table.put("DriveAppliedVolts", getDriveAppliedVolts());
		table.put("DriveCurrentAmps", getDriveCurrentAmps());
		table.put("TurnAbsolutePosition", getTurnAbsolutePosition());
		table.put("TurnRelativePosition", getTurnRelativePosition());
		table.put("TurnVelocity", getTurnVelocity());
		table.put("TurnAppliedVolts", getTurnAppliedVolts());
		table.put("TurnCurrentAmps", getTurnCurrentAmps());
	}

	@Override
	public void fromLog(LogTable table) {
		setConnected(table.get("Connected", isConnected()));
		setDrivePosition(table.get("DrivePosition", getDrivePosition()));
		setDriveVelocity(table.get("DriveVelocity", getDriveVelocity()));
		setDriveAppliedVolts(table.get("DriveAppliedVolts", getDriveAppliedVolts()));
		setDriveCurrentAmps(table.get("DriveCurrentAmps", getDriveCurrentAmps()));
		setTurnAbsolutePosition(table.get("TurnAbsolutePosition", getTurnAbsolutePosition()));
		setTurnRelativePosition(table.get("TurnRelativePosition", getTurnRelativePosition()));
		setTurnVelocity(table.get("TurnVelocity", getTurnVelocity()));
		setTurnAppliedVolts(table.get("TurnAppliedVolts", getTurnAppliedVolts()));
		setTurnCurrentAmps(table.get("TurnCurrentAmps", getTurnCurrentAmps()));
	}

	/**
	 * Clones the ModuleIOInfo object.
	 * @return ModuleIOInfo clone
	 */
	public ModuleIOInfoAutoLogged clone() {
		ModuleIOInfoAutoLogged copy = new ModuleIOInfoAutoLogged();
		copy.setConnected(this.isConnected());
		copy.setDrivePosition(this.getDrivePosition());
		copy.setDriveVelocity(this.getDriveVelocity());
		copy.setDriveAppliedVolts(this.getDriveAppliedVolts());
		copy.setDriveCurrentAmps(this.getDriveCurrentAmps());
		copy.setTurnAbsolutePosition(this.getTurnAbsolutePosition());
		copy.setTurnRelativePosition(this.getTurnRelativePosition());
		copy.setTurnVelocity(this.getTurnVelocity());
		copy.setTurnAppliedVolts(this.getTurnAppliedVolts());
		copy.setTurnCurrentAmps(this.getTurnCurrentAmps());
		return copy;
	}
}
