package frc.robot;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NetworkTablesTest extends TimedRobot {
    DoubleSubscriber aprilYaw;
    IntegerSubscriber aprilId;
    DoubleSubscriber noteYaw;
    DoubleSubscriber noteDist;

    public void robotInit() {
        DataLogManager.start();
        DataLogManager.log("Hello World");
        NetworkTableInstance inst = NetworkTableInstance.create();

        NetworkTable table = inst.getTable("datatable");
        aprilYaw = table.getDoubleTopic("april_yaw").subscribe(-1);
        aprilId = table.getIntegerTopic("april_id").subscribe(-1);
        noteYaw = table.getDoubleTopic("note_yaw").subscribe(-1);
        noteDist = table.getDoubleTopic("note_distance").subscribe(-1);
    }

    @Override
    public void teleopPeriodic() {
            double tagYaw = aprilYaw.get();
            long tagId = aprilId.get();
            double nYaw = noteYaw.get();
            double nDist = noteDist.get();

            DataLogManager.log("tag yaw: " + tagYaw + ", tag id: " + tagId + ", note yaw: " + nYaw + ", note dist: " + nDist);
            SmartDashboard.putNumber("tagYaw: ", tagYaw );
            SmartDashboard.putNumber("tagId: ", tagId );
            SmartDashboard.putNumber("noteYaw: ", nYaw );
            SmartDashboard.putNumber("noteDist: ", nDist );


    }
}
