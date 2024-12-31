
package frc.robot.systems.climber;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.motorWrappers.SparkMaxWrapper;
import frc.robot.Constants;

public class Mech2DSim {
    private final Mechanism2d mech = new Mechanism2d(100, 100);
    private final MechanismRoot2d root = mech.getRoot("arm", 50, 50);
    private final MechanismLigament2d arm1 = root.append(new MechanismLigament2d(
        "bar1", 30, 90, 10, new Color8Bit(Color.kPurple)
    ));
    private final MechanismLigament2d arm2 = arm1.append(new MechanismLigament2d(
        "bar2", 15,  0, 10, new Color8Bit(Color.kYellow)
    ));
    private final double targetPos = (90.0 / 360.0) * Constants.CANSPARK_CPR;
    private final SparkMaxWrapper motor = new SparkMaxWrapper(0, MotorType.kBrushless);

    public Mech2DSim() {
        motor.set(0.5);
    }

    public void update() {
        double pos = motor.getEncoderPosition();
        if (pos >= targetPos) motor.set(0);
        arm2.setAngle((pos / Constants.CANSPARK_CPR) * 360.0);
        Logger.recordOutput("Robot Simulation", mech);
    }
}
