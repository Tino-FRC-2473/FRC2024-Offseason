package frc.robot.systems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

    @AutoLog
    public class ModuleIOInfo {
        public double drivePosition = 0.0; // rad
        public double driveVelocity = 0.0; // rad/s

        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
    
        public Rotation2d turnAbsolutePosition = new Rotation2d(); //this is generated from the CANCoder 
        public Rotation2d turnRelativePosition = new Rotation2d(); //this is using the talonfx encoder val
        public double turnVelocity = 0.0;

        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
    }

    /** Update all the values of the logged values of the module above */
    public default void updateInputs(ModuleIOInfo moduleInfo) {}

    /** Run the drive motor at the specified voltage. */
    public default void setDriveVoltage(double volts) {}

    /** Run the turn motor at the specified voltage. */
    public default void setTurnVoltage(double volts) {}

    /** Enable or disable brake mode on the drive motor. */
    public default void setDriveBrakeMode(boolean enable) {}

    /** Enable or disable brake mode on the turn motor. */
    public default void setTurnBrakeMode(boolean enable) {}

    /** Set the shorthanded logging name of the module */
    public default void setModuleName(String moduleName) {}

    /** Get the shorthanded logging name of the module */
    public default String getModuleName() { return ""; }

    /** Reset the encoder position values. */
    public default void resetEncoders() {}

}