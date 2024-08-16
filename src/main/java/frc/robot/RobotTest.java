package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

public class RobotTest {
    public void robotInit() {
        NetworkTableInstance inst = NetworkTableInstance.create();

        NetworkTable table = inst.getTable("datatable");

        while(true) {
            double x = table.getDoubleTopic("x").getEntry(0).get();
            double y = table.getDoubleTopic("y").getEntry(0).get();

            System.out.println("x: " + x + ", y: " + y);
        }

    }

}
