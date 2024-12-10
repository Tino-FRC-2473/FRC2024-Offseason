package frc.robot.motorWrappers;
import java.util.*;
public interface Loggable {
    
    static final List<Loggable> motors = new ArrayList<>();

    public double get();
    public int getCanId();
    public double getEncoderPosition();

    default void update() {}
    default void init() { motors.add(this); }
    
}
