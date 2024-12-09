package frc.robot.systems.drive.module;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleIOInfoAutoLogged extends ModuleIO.ModuleIOInfo
	implements LoggableInputs, Cloneable {

	private String mID;

	/**
	 * ModuleIOInfoAutoLogged object.
	 * @param moduleID
	 */
	public ModuleIOInfoAutoLogged(String moduleID) {
		mID = moduleID;
	}

	@Override
	public void toLog(LogTable table) {
		table.put(mID + ": Connected", isConnected());
		table.put(mID + ": DrivePosition", getDrivePosition());
		table.put(mID + ": DriveVelocity", getDriveVelocity());
		table.put(mID + ": DriveAppliedVolts", getDriveAppliedVolts());
		table.put(mID + ": DriveCurrentAmps", getDriveCurrentAmps());
		table.put(mID + ": TurnAbsolutePosition", getTurnAbsolutePosition());
		table.put(mID + ": TurnRelativePosition", getTurnRelativePosition());
		table.put(mID + ": TurnVelocity", getTurnVelocity());
		table.put(mID + ": TurnAppliedVolts", getTurnAppliedVolts());
		table.put(mID + ": TurnCurrentAmps", getTurnCurrentAmps());
	}

	@Override
	public void fromLog(LogTable table) {
		setConnected(table.get(mID + ": Connected", isConnected()));
		setDrivePosition(table.get(mID + ": DrivePosition", getDrivePosition()));
		setDriveVelocity(table.get(mID + ": DriveVelocity", getDriveVelocity()));
		setDriveAppliedVolts(table.get(mID + ": DriveAppliedVolts", getDriveAppliedVolts()));
		setDriveCurrentAmps(table.get(mID + ": DriveCurrentAmps", getDriveCurrentAmps()));
		setTurnAbsolutePosition(table.get(mID
			+ ": TurnAbsolutePosition", getTurnAbsolutePosition()));
		setTurnRelativePosition(table.get(mID
			+ ": TurnRelativePosition", getTurnRelativePosition()));
		setTurnVelocity(table.get(mID + ": TurnVelocity", getTurnVelocity()));
		setTurnAppliedVolts(table.get(mID + ": TurnAppliedVolts", getTurnAppliedVolts()));
		setTurnCurrentAmps(table.get(mID + ": TurnCurrentAmps", getTurnCurrentAmps()));
	}

	/**
	 * Clones the ModuleIOInfo object.
	 * @return ModuleIOInfo clone
	 */
	public ModuleIOInfoAutoLogged clone() {
		ModuleIOInfoAutoLogged copy = new ModuleIOInfoAutoLogged(mID);
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
