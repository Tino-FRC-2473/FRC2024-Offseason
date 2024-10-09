package frc.robot;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	// public static final int CAN_ID_SPARK_SHOOTER_UPPER = 33;
	// public static final int CAN_ID_SPARK_SHOOTER_LOWER = 34;

	// RoboRIO Ports
	public static final int ENCODER_CHANNEL_A = 0;
	public static final int ENCODER_CHANNEL_B = 1;

	// ID numbers for devices on the CAN bus
	public static final int INTAKE_MOTOR_ID = 9; //9
	public static final int PIVOT_MOTOR_ID = 17; //17
	public static final int INDEXER_MOTOR_ID = 11; //11

	public static final int LEFT_SHOOTER_MOTOR_ID = 12; //0
	public static final int RIGHT_SHOOTER_MOTOR_ID = 13; //4

	public static final int RIGHT_CLIMBER_CAN_ID = 15;
	public static final int LEFT_CLIMBER_CAN_ID = 16;

	// NEW Chassis

	public static final int FRONT_LEFT_DRIVING_CAN_ID = 32;
	public static final int FRONT_RIGHT_DRIVING_CAN_ID = 28;
	public static final int REAR_LEFT_DRIVING_CAN_ID = 24;
	public static final int REAR_RIGHT_DRIVING_CAN_ID = 26;

	public static final int FRONT_LEFT_TURNING_CAN_ID = 31;
	public static final int FRONT_RIGHT_TURNING_CAN_ID = 27;
	public static final int REAR_LEFT_TURNING_CAN_ID = 23;
	public static final int REAR_RIGHT_TURNING_CAN_ID = 25;

	public static final int CAN_ID_SPARK_LEFT_CLIMBER_MOTOR = 19;
	public static final int CAN_ID_SPARK_RIGHT_CLIMBER_MOTOR = 20;

	// OLD Chassis

	// public static final int FRONT_LEFT_DRIVING_CAN_ID = 8;
	// public static final int FRONT_RIGHT_DRIVING_CAN_ID = 6;
	// public static final int REAR_LEFT_DRIVING_CAN_ID = 2;
	// public static final int REAR_RIGHT_DRIVING_CAN_ID = 4;

	// public static final int FRONT_LEFT_TURNING_CAN_ID = 7;
	// public static final int FRONT_RIGHT_TURNING_CAN_ID = 5;
	// public static final int REAR_LEFT_TURNING_CAN_ID = 1;
	// public static final int REAR_RIGHT_TURNING_CAN_ID = 3;

	public static final int CAN_ID_SPARK_LSHOOTER_MOTOR = 38;
	public static final int CAN_ID_SPARK_RSHOOTER_MOTOR = 36;
	public static final int DEVICE_ID_ARM_MOTOR = 0;
	public static final int DEVICE_ID_INTAKE_MOTOR = 1;
}
