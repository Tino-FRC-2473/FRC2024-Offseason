package frc.robot.motorWrappers;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants;

public class SparkMaxWrapper extends CANSparkMax implements Loggable {

    public SparkMaxWrapper(int deviceId, MotorType type) {
        super(deviceId, type);
        init();
    }

    public void update() {
        double delta = Constants.DEFAULT_PERIODIC_SECS;
        double encoderCPR = Constants.CANSPARK_CPR;
        
        // position += get() * delta * encoderCPR;
        getEncoder().setPosition(getEncoder().getPosition() + get() * delta * encoderCPR);
    }

    @Override
    public double getEncoderPosition() {
        return getEncoder().getPosition();
    }

    @Override
    public int getCanId() {
        return this.getDeviceId();
    }

}
