package frc.robot;

public final class Constants {

	// Auto Shooter Constants
	public static final double AUTO_REVVING_SECS = 0.5;
	public static final double AUTO_SHOOTING_SECS = 1.0;
	public static final double AUTO_PRELOAD_SHOOTING_SECS = 1.9;
	public static final double AUTO_PRELOAD_REVVING_SECS = 1.3;

	// TeleOp Shooter Constants
	public static final float SHOOTING_POWER = 0.8f;
	public static final float TIMED_REVVING_DURATION = 2;
	public static final float SHOOT_VELOCITY = 75; //must be <=89rps

	// Auto Intake Constants
	public static final double AUTO_PIVOT_TIMER = 0.5;
	public static final double OUTTAKE_AUTO_TIMER = 0.7;
	public static final float TIMED_INTAKING_DURATION = 1.5f;
	public static final float AUTO_INTAKE_POWER = 0.2f;
	public static final float AUTO_OUTTAKE_POWER = -0.2f;
	public static final float AUTO_HOLDING_POWER = 0.05f;


	// TeleOp Intake Constants
	public static final float INTAKE_VELOCITY = 40; //must be <=89rps
	public static final float OUTTAKE_VELOCITY = -40; //must be >=-89rps
	public static final float FEED_SHOOTER_VELOCITY = 55; //must be <=89rps

	// REV Color Sensor Constants
	public static final int NOTE_FRAMES_MIN = 1;
	public static final double PROXIMIIY_THRESHOLD = 150;

	public static final double GREEN_LOW = 0.18;
	public static final double BLUE_LOW = 0.00;
	public static final double RED_LOW = 0.54;

	public static final double GREEN_HIGH = 0.35;
	public static final double BLUE_HIGH = 0.1;
	public static final double RED_HIGH = 0.8;
	// PID Constants
		// TeleOp
	public static final double MIN_TURN_SPEED = -0.6;
	public static final double MAX_TURN_SPEED = 0.6;
	public static final double PID_CONSTANT_PIVOT_P = 0.0007;
	public static final double PID_CONSTANT_PIVOT_I = 0;
	public static final double PID_CONSTANT_PIVOT_D = 0;
		// Auto
	public static final double MIN_TURN_SPEED_AUTO = -0.6;
	public static final double MAX_TURN_SPEED_AUTO = 0.6;
	public static final double PID_CONSTANT_PIVOT_P_AUTO = 0.001;
	public static final double INRANGE_VALUE = 30;

	// Encoder Position Constants
	public static final double GROUND_ENCODER_COUNT = -540;
	public static final double HOME_ENCODER_COUNT = 0;

	// Motion Magic Constants
	public static final double MM_CONSTANT_S = 0.15; //Voltage required to overcome static friction
	public static final double MM_CONSTANT_V = 0.014; //Voltage for velocity of 1rps
	public static final double MM_CONSTANT_A = 0.01; //Voltage for acceleration of 1rps/s
	public static final double MM_CONSTANT_P = 0.10; //Voltgae for Proportional error of 1 rps
	public static final double MM_CONSTANT_I = 0.001; //Voltage for Integrated error of 1 rps
	public static final double MM_CONSTANT_D = 0.001; //Voltage for Integrated error of 1 rps

	public static final double CONFIG_CONSTANT_A = 400; //max acceleration in rps/s
	public static final double CONFIG_CONSTANT_J = 4000; //target jerk in rps/s/s

	// Controller Rumble Power
	public static final float SOFT_RUMBLE = 0.5f;
	public static final float HARD_RUMBLE = 1.0f;
}
