package frc.robot;

public final class Constants {
	// PID Constants
	public static final double INRANGE_VALUE = 0.5;

	public static final double TARGET_VELO_RPS = 84;

	public static final double SLOT_0_S = 0.1; //0.10
		//Voltage required to overcome static friction (0.15)
	public static final double SLOT_0_V = 0.12; // 0.1
		//Voltage for velocity of 1rps (0.1)
	public static final double SLOT_0_A = 0.01; // 0.01
		//Voltage for acceleration of 1rps/s
	public static final double SLOT_0_P = 0.0; // 0.9
		//Voltgae for Proportional error of 1 rot(0.7)
	public static final double SLOT_0_I = 0.0;
		//Voltage for Integrated error of 1 r*s
	public static final double SLOT_0_D = 0.0;
		//Voltage for Derivative error of 1 rps

	public static final double MMAGIC_CONSTANT_A = 356; // Max acceleration in rps/s (0.25 seconds)
	public static final double MMAGIC_CONSTANT_J = 3560; // Target jerk in rps/s/s (0.1 seconds)

	// Other
	public static final int UPDATE_FREQUENCY_HZ = 200;
}
