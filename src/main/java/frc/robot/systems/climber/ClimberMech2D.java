package frc.robot.systems.climber;

import frc.robot.Constants;
import frc.robot.HardwareMap;
import frc.robot.motorWrappers.CANSparkMaxWrapper;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;

public class ClimberMech2D {
    private final CANSparkMaxWrapper leftMotor, rightMotor;
    private final Mechanism2d mechanism2d;
    private final MechanismLigament2d leftClimber, rightClimber;

    public ClimberMech2D() {
        leftMotor = new CANSparkMaxWrapper(
            HardwareMap.LEFT_CLIMBER_CAN_ID, 
            CANSparkMaxWrapper.MotorType.kBrushless); 
        rightMotor = new CANSparkMaxWrapper(
			HardwareMap.RIGHT_CLIMBER_CAN_ID,
			CANSparkMax.MotorType.kBrushless);        
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);

        mechanism2d = new Mechanism2d(100, 100); 
        MechanismRoot2d rootLeft = mechanism2d.getRoot("LeftClimber", 20, 50); 
        MechanismRoot2d rootRight = mechanism2d.getRoot("RightClimber", 80, 50); 
        leftClimber = rootLeft.append(new MechanismLigament2d("Left Arm", 30, 90)); 
        rightClimber = rootRight.append(new MechanismLigament2d("Right Arm", 30, 90)); 

        Logger.recordOutput("ClimberMechanism2D", mechanism2d);
    }

    public void update() {
        double leftEncoderPos = leftMotor.getEncoder().getPosition();
        double rightEncoderPos = rightMotor.getEncoder().getPosition();

        double leftAngle = encoderToAngle(leftEncoderPos);
        double rightAngle = encoderToAngle(rightEncoderPos);

        leftClimber.setAngle(leftAngle);
        rightClimber.setAngle(rightAngle);

        Logger.recordOutput("Climber/LeftClimberAngle", leftAngle);
        Logger.recordOutput("Climber/RightClimberAngle", rightAngle);
    }

    private double encoderToAngle(double encoderPosition) {
        return (encoderPosition / Constants.SPARK_CPR) * 360.0;
    }
}
