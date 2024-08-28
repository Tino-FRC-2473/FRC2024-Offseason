package frc.robot;

public final class MechConstants {

	// Auto Shooter Constants
	public static final double AUTO_REVVING_TIME = 0.5;
	public static final double AUTO_SHOOTING_TIME = 1.0;
	public static final double AUTO_PRELOAD_SHOOTING_TIME = 1.9;
	public static final double AUTO_PRELOAD_REVVING_TIME = 1.3;

	// TeleOp Shooter Constants
	public static final float SHOOTING_POWER = 0.8f;
	public static final float TIMED_REVVING_DURATION = 2;
	public static final float SHOOT_VELOCITY = 100; // rps

	// Auto Intake Constants
	public static final double AUTO_PIVOT_TIMER = 0.5;
	public static final double OUTTAKE_AUTO_TIMER = 0.7;
	public static final float TIMED_INTAKING_DURATION = 1.5f;
	public static final float AUTO_INTAKE_POWER = 0.2f;
	public static final float AUTO_OUTTAKE_POWER = -0.2f;
	public static final float AUTO_HOLDING_POWER = 0.05f;

	// TeleOp Intake Constants
	public static final float INTAKE_POWER = 0.2f; //0.4
	public static final float OUTTAKE_POWER = -0.2f;
	public static final float FEED_SHOOTER_POWER = -0.8f;
	public static final float MANUAL_INTAKE_POWER = 0.2f;
	public static final float MANUAL_OUTTAKE_POWER = -0.2f;
	public static final float TELE_HOLDING_POWER = 0.0f;

	// REV Color Sensor Constants
	public static final int NOTE_FRAMES_MIN = 5;
	public static final double PROXIMIIY_THRESHOLD = 200;

	public static final double GREEN_LOW = 0.18;
	public static final double BLUE_LOW = 0.00;
	public static final double RED_LOW = 0.54;

	public static final double GREEN_HIGH = 0.35;
	public static final double BLUE_HIGH = 0.1;
	public static final double RED_HIGH = 0.8;
// TODO: TESTS ALL CONSTANTS AND FIX
	// PID Constants
		// TeleOp
	public static final double MIN_TURN_SPEED = -0.4;
	public static final double MAX_TURN_SPEED = 0.4;
	public static final double PID_CONSTANT_PIVOT_P = 0.00075;
		// Auto
	public static final double MIN_TURN_SPEED_AUTO = -0.6;
	public static final double MAX_TURN_SPEED_AUTO = 0.6;
	public static final double PID_CONSTANT_PIVOT_P_AUTO = 0.001;
	public static final double INRANGE_VALUE = 30;

	// Encoder Position Constants
	public static final double GROUND_ENCODER_ROTATIONS = -1200;
	public static final double AMP_ENCODER_ROTATIONS = -525;
	public static final double SHOOTER_ENCODER_ROTATIONS = 0;
	public static final double HOME_ENCODER_ROTATION = 0;

	// Motion Magic Constants
	public static final double MM_CONSTANT_S = 0.25; //0.25V to overcome static friction
	public static final double MM_CONSTANT_V = 0.114; //0.114V for 1rps target velocity
	public static final double MM_CONSTANT_A = 0.01; //0.01V for 1rps/s acceleration
	public static final double MM_CONSTANT_P = 0.11; //0.11V to account for error of 1rps
	public static final double MM_CONSTANT_I = 0.002; //0.002V Integrated error 1 rps

	public static final double CONFIG_CONSTANT_A = 400; //max acceleration of 400 rps/s
	public static final double CONFIG_CONSTANT_J = 4000; //target jerk of 4000 rps/s/s
}
