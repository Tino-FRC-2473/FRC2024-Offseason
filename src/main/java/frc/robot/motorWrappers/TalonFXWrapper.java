package frc.robot.motorWrappers;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;


public class TalonFXWrapper extends TalonFX {

    TalonFXSimState sim;

    public TalonFXWrapper(int deviceId) { 
        super(deviceId); 
        sim = getSimState();
    }

    public TalonFXWrapper(int deviceId, String canbus) { 
        super(deviceId,canbus); 
        sim = getSimState();
    }

   @Override 
    public void set(double speed) {
        super.set(speed);
        if (Robot.isSimulation()) sim.setRotorVelocity(speed);
    }

    @Override
    public StatusCode setControl(MotionMagicVelocityVoltage request) {
        if (Robot.isSimulation()) sim.setRotorVelocity(request.Velocity);
        SmartDashboard.putNumber("Speed of ID "+getDeviceID(), get());
        Logger.recordOutput("Speed of ID "+getDeviceID(),get());
        return super.setControl(request);
    }
}
