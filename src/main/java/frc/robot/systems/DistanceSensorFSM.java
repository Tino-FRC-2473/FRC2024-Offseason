
//We might not even need an FSM for the Distance Sensor since its just a few methods?
package frc.robot.systems;
import org.opencv.objdetect.CascadeClassifier;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class DistanceSensorFSM {
    //import the Analog Library


    //Create the Analog Object
    private AnalogInput sharp;

    //Measured in CM
    private double distStart = 15;
    private double distEnd = 30;
    //Constuct a new instance
    public DistanceSensorFSM(){
        sharp = new AnalogInput(0);
        reportToDashboard();
    }
    

    //Create an accessor method
    public double getDistance(){
        return (Math.pow(sharp.getAverageVoltage(), -1.2045)) * 27.726;
    }
    public void reportToDashboard(){
        
        double distance = getDistance();
        if(distance < distStart){
            SmartDashboard.putBoolean("Less than distance?", true);
        }else if (distance >= distStart && distance <= distEnd){
            SmartDashboard.putBoolean("Less than distance?", false);
        }else{
            return;
        }
        String save = "" + distance + "";
        SmartDashboard.putString("distance is", save);
        SmartDashboard.updateValues();
    }
    
    
}
