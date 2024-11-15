package frc.robot;

public final class Constants {
	// PID Constants
	public static final double INRANGE_VALUE = 30;

	// Encoder Position Constants
	public static final double DEPLOYED_POSITION = 540;
	public static final double AMP_POSITION = DEPLOYED_POSITION / 2;
	public static final double HOME_POSITION = 0;

	// Motion Magic Constants
	public static final double MM_CONSTANT_G = 0.0;
	public static final double MM_CONSTANT_S = 0.2; //Voltage required to overcome static friction
	public static final double MM_CONSTANT_V = 0.1; //Voltage for velocity of 1rps
	public static final double MM_CONSTANT_A = 0.01; //Voltage for acceleration of 1rps/s
	public static final double MM_CONSTANT_P = 0.0007; //Voltgae for Proportional error of 1 rps
	public static final double MM_CONSTANT_I = 0.00; //Voltage for Integrated error of 1 rps
	public static final double MM_CONSTANT_D = 0.00; //Voltage for Integrated error of 1 rps

	public static final double CONFIG_CONSTANT_CV = 30; // Cruise Velo in rps
	public static final double CONFIG_CONSTANT_A = 160; //max acceleration in rps/s
	public static final double CONFIG_CONSTANT_J = 1600; //target jerk in rps/s/s

	// Other
	public static final int UPDATE_FREQUENCY_HZ = 200;
}
