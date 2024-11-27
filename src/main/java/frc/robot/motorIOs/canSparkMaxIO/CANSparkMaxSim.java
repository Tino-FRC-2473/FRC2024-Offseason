package frc.robot.motorIOs.canSparkMaxIO;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;

public class CANSparkMaxSim implements CANSparkMaxIO {

    @Override
    public void setVoltage(double outputVolts) {
        
        System.out.println("this is a test");
        Logger.recordOutput(String.format("voltage for %s:",toString()),outputVolts);
    }

    @Override
    public void set(double speed) {
        throw new UnsupportedOperationException("Unimplemented method 'set'");
    }

    @Override
    public double get() {
        throw new UnsupportedOperationException("Unimplemented method 'get'");
    }
}
