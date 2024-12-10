package frc.robot.motorWrappers;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class TalonFXWrapper extends TalonFX implements Loggable {

    public TalonFXWrapper(int deviceId) {
        super(deviceId);
        init();
    }

    public TalonFXWrapper(int deviceId, String canbus) {
        super(deviceId, canbus);
        init();
    }

    public void update() {
        double delta = Constants.DEFAULT_PERIODIC_SECS;
        double encoderCPR = Constants.TALONFX_CPR;

        setPosition(getPosition().getValue() + get() * delta * encoderCPR);
    }

    @Override
    public void set(double speed) {
        super.set(speed);
    }

    @Override
    public int getCanId() {
        return this.getDeviceID();
    }

    @Override
    public double getEncoderPosition() {
        return getPosition().getValue();
    }
}
