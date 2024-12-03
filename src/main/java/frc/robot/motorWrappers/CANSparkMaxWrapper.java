package frc.robot.motorWrappers;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.sim.SparkRelativeEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CANSparkMaxWrapper extends CANSparkMax {
    //private SparkRelativeEncoderSim sim;
    

    public CANSparkMaxWrapper(int deviceId, MotorType type) {
        super(deviceId,type);

    }

    @Override 
    public void set(double speed) {
        super.set(speed);
        SmartDashboard.putNumber("Speed of ID "+getDeviceId(), get());
        Logger.recordOutput("Speed of ID "+getDeviceId(),get());
        getEncoder().setPosition(getEncoder().getPosition() + speed);
    }
}