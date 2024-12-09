package frc.robot.motorWrappers;

import java.util.*;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

public class CANSparkMaxWrapper extends CANSparkMax {

    private static List<CANSparkMaxWrapper> objs = new ArrayList<>();
    private double lastSpeed; 
    private double position; 

    public CANSparkMaxWrapper(int deviceId, MotorType type) {
        super(deviceId, type);
        objs.add(this); 
    }

    public void update() {
        double deltaTime = Constants.DEFAULT_PERIODIC_SECS;
        double encoderCPR = Constants.ENCODER_COUNTS_PER_REV; 
        // double gearRatio = Constants.GEAR_RATIO; // currently assume 1:1, else divide position by this
        position += lastSpeed * deltaTime * encoderCPR;
        getEncoder().setPosition(position);
        SmartDashboard.putNumber("Encoder Position of Motor ID " + getDeviceId(), position);
        Logger.recordOutput("Encoder Position of Motor ID " + getDeviceId(), position);
    }

    @Override
    public void set(double speed) {
        super.set(speed);
        lastSpeed = speed; 
        SmartDashboard.putNumber("Motor Speed of ID " + getDeviceId(), speed);
        Logger.recordOutput("Motor Speed of ID " + getDeviceId(), speed);
    }
    public static void init() {
        objs.clear();
    }

    public static void updateAll() {
        for (CANSparkMaxWrapper obj : objs) {
            obj.update();
        }
    }
}
