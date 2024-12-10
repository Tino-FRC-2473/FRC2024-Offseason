package frc.robot.motorWrappers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorManager {

    public static void update() {
        for (Loggable l : Loggable.motors) {
            l.update();
            SmartDashboard.putNumber("Velocity " + l.getCanId(), l.get());
            SmartDashboard.putNumber("Position " + l.getCanId(), l.getEncoderPosition());
            Logger.recordOutput("Velocity " + l.getCanId(), l.get());
            Logger.recordOutput("Position " + l.getCanId(), l.getEncoderPosition());
        }
    }

}
