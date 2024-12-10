package frc.robot.motorWrappers;
import java.util.*;
public interface Loggable {
    public double get();
    public int getCanId();
    public double getEncoderPosition();
    default void update() {}
    static final List<Loggable> motors = new ArrayList<>();
    default void init() { motors.add(this); }
}
