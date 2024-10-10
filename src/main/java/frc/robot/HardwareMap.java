package frc.robot;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {

	// RoboRIO Ports
	public static final int ENCODER_CHANNEL_A = 0;
	public static final int ENCODER_CHANNEL_B = 1;

	// ID numbers for devices on the CAN bus
	public static final int INTAKE_MOTOR_ID = 9; //9
	public static final int PIVOT_MOTOR_ID = 17; //17
	public static final int INDEXER_MOTOR_ID = 11; //11

	public static final int LEFT_SHOOTER_MOTOR_ID = 12; //12
	public static final int RIGHT_SHOOTER_MOTOR_ID = 13; //13

	public static final int RIGHT_CLIMBER_CAN_ID = 15;
	public static final int LEFT_CLIMBER_CAN_ID = 16;

	// NEW Chassis

	public static final int FRONT_LEFT_DRIVING_CAN_ID = 28;
	public static final int FRONT_RIGHT_DRIVING_CAN_ID = 32;
	public static final int REAR_LEFT_DRIVING_CAN_ID = 26;
	public static final int REAR_RIGHT_DRIVING_CAN_ID = 24;

	public static final int FRONT_LEFT_TURNING_CAN_ID = 27;
	public static final int FRONT_RIGHT_TURNING_CAN_ID = 31;
	public static final int REAR_LEFT_TURNING_CAN_ID = 25;
	public static final int REAR_RIGHT_TURNING_CAN_ID = 23;

	// OLD Chassis

	// public static final int FRONT_LEFT_DRIVING_CAN_ID = 8;
	// public static final int FRONT_RIGHT_DRIVING_CAN_ID = 6;
	// public static final int REAR_LEFT_DRIVING_CAN_ID = 2;
	// public static final int REAR_RIGHT_DRIVING_CAN_ID = 4;

	// public static final int FRONT_LEFT_TURNING_CAN_ID = 7;
	// public static final int FRONT_RIGHT_TURNING_CAN_ID = 5;
	// public static final int REAR_LEFT_TURNING_CAN_ID = 1;
	// public static final int REAR_RIGHT_TURNING_CAN_ID = 3;
}
