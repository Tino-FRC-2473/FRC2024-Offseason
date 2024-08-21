package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;

public class RobotTest extends TimedRobot{
    DoubleSubscriber xSub;
    DoubleSubscriber ySub;


    public void robotInit() {
        DataLogManager.start();
        DataLogManager.log("Hello world!");
        System.out.println("Hello world!");
        NetworkTableInstance inst = NetworkTableInstance.create();

        NetworkTable table = inst.getTable("datatable");
        xSub = table.getDoubleTopic("testX").subscribe(-3);
        ySub = table.getDoubleTopic("testY").subscribe(-3);

    }

    @Override
    public void teleopPeriodic() {
            double x = xSub.get();
            double y = ySub.get();

            DataLogManager.log("x: " + x + ", y: " + y);
            SmartDashboard.putNumber("x: ", x );
            SmartDashboard.putNumber("y: ", y );


    }
}
