package frc.robot.motorWrappers;
import java.util.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CANSparkMaxWrapper extends CANSparkMax {

    private static List<CANSparkMaxWrapper> objs = new ArrayList<>();

    public CANSparkMaxWrapper(int deviceId, MotorType type) {
        super(deviceId,type);
    }

    public void update() {
        var encoder = getEncoder();
        encoder.setPosition(encoder.getPosition()+encoder.getCountsPerRevolution()*get());
    }

    @Override 
    public void set(double speed) {
        super.set(speed);
        SmartDashboard.putNumber("Speed of ID "+getDeviceId(), get());
        Logger.recordOutput("Speed of ID "+getDeviceId(),get());
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