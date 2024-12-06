
//We might not even need an FSM for the Distance Sensor since its just a few methods?
package frc.robot.systems;
import org.opencv.objdetect.CascadeClassifier;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.DeployerFSM.DeployerFSMState;
public class DistanceSensorFSM {
    //import the Analog Library



    //Create the Analog Object
    private AnalogInput sharp;

    //Measured in CM
    private double distStart = 15;
    private double distEnd = 30;
    private final double POWER_FOR_VOLTAGE = -1.2045;
    private final double DISTANCE_MULTIPLE = 27.726;

    //create FSM state
    private SensorTestingFSMState currentState;
    public enum SensorTestingFSMState{
        OUT_OF_RANGE,IN_RANGE
    }

    //Constuct a new instance
    public DistanceSensorFSM(){
        currentState = SensorTestingFSMState.OUT_OF_RANGE;
        sharp = new AnalogInput(0);
        update(sharp);
        reportToDashboard();
    }
    //Create an accessor method
    public double getDistance(){
        return (Math.pow(sharp.getAverageVoltage(), POWER_FOR_VOLTAGE)) * DISTANCE_MULTIPLE;
    }
    public void reset(){
        currentState = SensorTestingFSMState.OUT_OF_RANGE;

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
    public void update(AnalogInput input){
        boolean isInRange;
        if(getDistance() < distStart || getDistance() > distEnd){
            isInRange = false;
        }else{
            isInRange = true;
        }
        switch(currentState){
            case OUT_OF_RANGE:
                if(!isInRange){
                    currentState = SensorTestingFSMState.OUT_OF_RANGE;
                    handleOutOfRange();
                }
                break;
            case IN_RANGE:
                if(isInRange){
                    currentState = SensorTestingFSMState.IN_RANGE;
                    handleInRange();
                }
                break;
            default:
                break;
        }

    
    }
    public void handleInRange(){
        String distString = "" + getDistance() + "";
        SmartDashboard.putString("In range of distance sensor value in mm: ", distString);
    }
    public void handleOutOfRange(){
        String distString = "" + getDistance() + "";
        SmartDashboard.putString("Out of range value in mm", distString);
    }
    
    
    
}
