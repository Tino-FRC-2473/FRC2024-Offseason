package frc.robot.systems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// FSM state definitions
import frc.robot.TeleopInput;
import frc.robot.systems.DeployerFSM.DeployerFSMState;
	
public class BreakBeamSensorFSM {
    private DigitalInput breakBeam;
    private SensorTestingFSMState state;
    public enum SensorTestingFSMState {
		BEAM_INTACT,
		BEAM_BROKEN
	}
    public BreakBeamSensorFSM(){
        breakBeam = new DigitalInput(0);
        state = SensorTestingFSMState.BEAM_INTACT;
        update(breakBeam);
    }
    public void update(DigitalInput input){
        boolean isBeamBroken = breakBeam.get();
        
        if(input == null){
            return;
        }
        switch (state) {
            case BEAM_INTACT:
                if(isBeamBroken == true) {
                    state = SensorTestingFSMState.BEAM_BROKEN;
                    handleBeamBroken();
                }
                //handleFirstInput();
                break;
            case BEAM_BROKEN:
                if(isBeamBroken == false){
                    state = SensorTestingFSMState.BEAM_INTACT;
                    handleBeamNotBroken();
                }
                break;
                //handleSecondInput();
        
            default:
                break;
        }
        //state =(input);
        SmartDashboard.putBoolean("Break beam value", breakBeam.get());
    }
    public void handleBeamBroken(){
        SmartDashboard.putString("Beam status", "true");
    }
    public void handleBeamNotBroken(){
        SmartDashboard.putString("Beam status", "false");
    }
    public boolean isBeamBroken() {
        if(state == SensorTestingFSMState.BEAM_BROKEN){
            return true;
        }
        return false;
    }
    public void reset(){
        state = SensorTestingFSMState.BEAM_INTACT;
        update(null);

    }

}