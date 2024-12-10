package frc.robot.motorWrappers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorLogger {
    public static void update() {
        for (Loggable l : Loggable.motors) {
            l.update();
            SmartDashboard.putNumber("Velocity "+l.getCanId(), l.get());
            SmartDashboard.putNumber("Position "+l.getCanId(),l.get());
        }
    }
}
