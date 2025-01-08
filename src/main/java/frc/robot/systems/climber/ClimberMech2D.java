
package frc.robot.systems.climber;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;

public class ClimberMech2D {
    private final Mechanism2d mech;
    private final MechanismLigament2d leftClimber, rightClimber;

    public ClimberMech2D() {

        mech = new Mechanism2d(100, 100); 
        MechanismRoot2d leftRoot = mech.getRoot("LeftClimber", 20, 50); 
        leftClimber = leftRoot.append(new MechanismLigament2d("Left Arm", 30, 90)); 
        MechanismRoot2d rightRoot = mech.getRoot("RightClimber", 80, 50); 
        rightClimber = rightRoot.append(new MechanismLigament2d("Right Arm", 30, 90)); 

        Logger.recordOutput("ClimberMechanism2D", mech);

    }

    public void update(double leftEncoderPos, double rightEncoderPos) {

        double leftAngle = encoderToAngle(leftEncoderPos);
        double rightAngle = encoderToAngle(rightEncoderPos);

        leftClimber.setLength(-leftEncoderPos + 100);
        rightClimber.setLength(rightEncoderPos + 100);

        Logger.recordOutput("Climber/LeftClimberAngle", leftAngle);
        Logger.recordOutput("Climber/RightClimberAngle", rightAngle);
        Logger.recordOutput("Climber/Mechanism2D", mech);
        SmartDashboard.putData(mech);
    }

    private double encoderToAngle(double encoderPosition) {
        return (encoderPosition / Constants.CANSPARK_CPR) * 360.0;
    }
}