package frc.robot.systems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInfo {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocity = 0.0;
  }

  /** Update a set of loggable inputs. */
  public default void updateInputs(GyroIOInfo inputs) {}

  /** Reset the heading of the gyro. */
  public default void resetHeading() {}
}