package frc.robot.motorIOs.canSparkMaxIO;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CANSparkMaxWrapper extends CANSparkMax {
    int deviceId;
    public CANSparkMaxWrapper(int deviceId, MotorType type) {
        super(deviceId,type);
        this.deviceId = deviceId;
    }

    @Override 
    public void set(double speed) {
        super.set(speed);
        SmartDashboard.putNumber("Speed of ID "+deviceId, get());
        getEncoder().setPosition(getEncoder().getPosition() + speed);
    }
}