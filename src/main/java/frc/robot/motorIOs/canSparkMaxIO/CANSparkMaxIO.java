package frc.robot.motorIOs.canSparkMaxIO;

public interface CANSparkMaxIO {
    public void setVoltage(double outputVolts);
    public void set(double speed);
    public double get();
}
