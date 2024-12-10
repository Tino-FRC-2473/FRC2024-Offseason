package frc.robot.motorWrappers;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import frc.robot.Robot;

public class CANSparkMaxWrapper extends CANSparkMax implements Loggable {

    private double position; 

    public CANSparkMaxWrapper(int deviceId, MotorType type) {
        super(deviceId, type); 
        init();
    }

    public void update() {
        double deltaTime = Constants.DEFAULT_PERIODIC_SECS;
        double encoderCPR = getEncoder().getCountsPerRevolution();
        position += get() * deltaTime * encoderCPR;
        getEncoder().setPosition(position);
    }

    @Override
    public double getEncoderPosition() {
        if (Robot.isReal()) return getEncoder().getPosition();
        else return position;
    }

    @Override
    public int getCanId() { 
        return this.getDeviceId();
    }
}
