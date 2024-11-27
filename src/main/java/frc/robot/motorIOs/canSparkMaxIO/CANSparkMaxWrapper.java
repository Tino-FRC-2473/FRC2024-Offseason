package frc.robot.motorIOs.canSparkMaxIO;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class CANSparkMaxWrapper extends CANSparkMax implements CANSparkMaxIO {
    int deviceId;
    public CANSparkMaxWrapper(int deviceId, MotorType type) {
        super(deviceId,type);
        this.deviceId = deviceId;
        setVoltage(0);
    }

    @Override 
    public void setVoltage(double outputVolts) {
        Logger.recordOutput("Voltages/"+this,outputVolts);
        SmartDashboard.putNumber("Voltage for ID "+deviceId, outputVolts);
        if (Robot.isReal()) {
            super.setVoltage(outputVolts);
        } else {
            
        }
    }
}