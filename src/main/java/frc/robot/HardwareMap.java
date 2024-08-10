package frc.robot;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	// TODO : CHANGE ALL CAN AND DEVICE ID TO THE CORRECT ID NUMBERS
	public static final int LEFT_CLIMBER_CAN_ID = 19;
	public static final int RIGHT_CLIMBER_CAN_ID = 20;

	public static final int PIVOT_MOTOR_ID = 0;
	public static final int TOP_INTAKE_MOTOR_ID = 1;
	public static final int RIGHT_SHOOTER_MOTOR_ID = 2;
	public static final int LEFT_SHOOTER_MOTOR_ID = 3;
	public static final int BOTTOM_INTAKE_MOTOR_ID = 4;
	public static final int FRONT_INDEXER_MOTOR_ID = 5;
	public static final int BACK_INDEXER_MOTOR_ID = 6;

	// NEW Chassis

	public static final int FRONT_LEFT_DRIVING_CAN_ID = 2;
	public static final int FRONT_RIGHT_DRIVING_CAN_ID = 8;
	public static final int REAR_LEFT_DRIVING_CAN_ID = 4;
	public static final int REAR_RIGHT_DRIVING_CAN_ID = 6;

	public static final int FRONT_LEFT_TURNING_CAN_ID = 1;
	public static final int FRONT_RIGHT_TURNING_CAN_ID = 7;
	public static final int REAR_LEFT_TURNING_CAN_ID = 3;
	public static final int REAR_RIGHT_TURNING_CAN_ID = 5;

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
