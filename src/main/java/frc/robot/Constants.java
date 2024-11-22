package frc.robot;

public final class Constants {
	// PID Constants
	public static final double INRANGE_VALUE = 0.5;

	// Encoder Position Constants
	public static final double DEPLOYED_POSITION = -11.5;
	public static final double AMP_POSITION = -6.5;
	public static final double HOME_POSITION = 0;

	// Motion Magic Constants
	public static final double MM_CONSTANT_G = 0.17;
	public static final double MM_CONSTANT_S = 0.15; //Voltage required to overcome static friction
	public static final double MM_CONSTANT_V = 0.0; //Voltage for velocity of 1rps (0.1)
	public static final double MM_CONSTANT_A = 0.0; //Voltage for acceleration of 1rps/s (0.01)
	public static final double MM_CONSTANT_P = 0.0; //Voltgae for Proportional error of 1 rps
	public static final double MM_CONSTANT_I = 0.0; //Voltage for Integrated error of 1 rps
	public static final double MM_CONSTANT_D = 0.0; //Voltage for Integrated error of 1 rps

	public static final double CONFIG_CONSTANT_CV = 30; // Cruise Velo in rps
	public static final double CONFIG_CONSTANT_A = 160; //max acceleration in rps/s
	public static final double CONFIG_CONSTANT_J = 1600; //target jerk in rps/s/s

	// Other
	public static final int UPDATE_FREQUENCY_HZ = 200;
}
