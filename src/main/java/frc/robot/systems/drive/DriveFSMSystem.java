package frc.robot.systems.drive;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;

// WPILib Imports

// Third party Hardware Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Units;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.SwerveConstants.DriveConstants;
import frc.robot.SwerveConstants.OIConstants;
import frc.robot.systems.drive.gyro.GyroIO;
import frc.robot.systems.drive.gyro.GyroIOPigeon2;
import frc.robot.systems.drive.module.ModuleIO;
import frc.robot.systems.drive.module.Module;
import frc.robot.SwerveConstants.AutoConstants;
import frc.robot.systems.drive.gyro.GyroIOInfoAutoLogged;

public class DriveFSMSystem extends SubsystemBase {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELEOP_STATE
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	// The gyro sensor
	private GyroIO gyroIO = new GyroIOPigeon2();
	private GyroIOInfoAutoLogged gyroIOInfo = new GyroIOInfoAutoLogged();
	private Rotation2d rawGyroRotation = new Rotation2d();

	// Create list of modules and according module info with ModuleIO's
	private Module flModule;
	private Module frModule;
	private Module blModule;
	private Module brModule;

	private SwerveModulePosition[] lastModulePositions;
	private final SysIdRoutine sysId;

	//private RaspberryPI rpi = new RaspberryPI(); << not including for example

	// Odometry class for tracking robot pose
	private SwerveDriveOdometry odometry;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 *
	 * @param gyroPigeonIO
	 * @param flModuleIO
	 * @param frModuleIO
	 * @param blModuleIO
	 * @param brModuleIO
	 */
	public DriveFSMSystem(
		GyroIO gyroPigeonIO,
		ModuleIO flModuleIO,
		ModuleIO frModuleIO,
		ModuleIO blModuleIO,
		ModuleIO brModuleIO
	) {
		// Perform hardware init
		this.gyroIO = gyroPigeonIO;
		flModule = new Module(flModuleIO);
		frModule = new Module(frModuleIO);
		blModule = new Module(blModuleIO);
		brModule = new Module(brModuleIO);

		lastModulePositions = new SwerveModulePosition[] {
			new SwerveModulePosition(),
			new SwerveModulePosition(),
			new SwerveModulePosition(),
			new SwerveModulePosition()
		};

		odometry = new SwerveDriveOdometry(
				DriveConstants.DRIVE_KINEMATICS,
				rawGyroRotation,
				lastModulePositions,
				new Pose2d()
		);

		//Initialize Holonomic System in AutoBuilder + Pathplanner logging
		AutoBuilder.configureHolonomic(
				this::getPose,
					// Robot pose supplier
				this::setPose,
					// Method to reset odometry (will be called if your auto has a starting pose)
				this::getRobotRelativeSpeeds,
					// ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::runVelocity,
					// Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig(
						// HolonomicPathFollowerConfig, this should live in your Constants class
						new PIDConstants(AutoConstants.AUTO_TRANSLATIONAL_KP,
							0.0, 0.0), // Translation PID const
						new PIDConstants(AutoConstants.AUTO_ROTATIONAL_KP,
							0.0, 0.0), // Rotation PID const
						AutoConstants.MAX_MODULE_SPEED, // Max module speed, in m/s
						AutoConstants.DRIVEBASE_RADIUS, // Drive base radius (in m).
						new ReplanningConfig() // Default path replanning config.
				),
			() ->
				DriverStation.getAlliance().isPresent()
				&& DriverStation.getAlliance().get() == Alliance.Red,
			this);

		Pathfinding.setPathfinder(new LocalADStar());
		PathPlannerLogging.setLogActivePathCallback(
			(activePath) -> {
				Logger.recordOutput(
					"Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])
				);
			}
		);
		PathPlannerLogging.setLogTargetPoseCallback(
			(targetPose) -> {
				Logger.recordOutput(
					"Odometry/Trajectory_Setpoint", targetPose
				);
			}
		);

		//SysId configuration
		sysId =
				new SysIdRoutine(
					new SysIdRoutine.Config(
						null,
						null,
						null,
						(state) -> Logger.recordOutput("Drive/SysIdState", state.toString())
					),
					new SysIdRoutine.Mechanism(
						(voltage) -> {
							flModule.runCharacterization(voltage.in(Units.Volts));
							frModule.runCharacterization(voltage.in(Units.Volts));
							blModule.runCharacterization(voltage.in(Units.Volts));
							brModule.runCharacterization(voltage.in(Units.Volts));
						},
						null,
						this
					)
				);

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */

	public void reset() {
		currentState = FSMState.TELEOP_STATE;

		setPose(new Pose2d());
		gyroIO.resetHeading();

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */

	public void resetAutonomous() {
		//currentState = FSMState.AUTO_STATE;

		setPose(getPose());
		gyroIO.resetHeading();

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers and runs general update protocols.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot's outputs are being reset.
	 */
	public void update(TeleopInput input) {

		//Refresh all the values from the StatusSignal + AdvKit logging
		flModule.processInputs();
		frModule.processInputs();
		blModule.processInputs();
		brModule.processInputs();
		gyroIO.updateInputs(gyroIOInfo);

		if (input == null) {
			resetEncoders();
			System.out.println("Encoders Reset");
			return;
		}

		if (DriverStation.isDisabled()) {
			Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
			Logger.recordOutput("SwerveStates/Setpoints_Optimized", new SwerveModuleState[] {});

			flModule.stop();
			frModule.stop();
			blModule.stop();
			brModule.stop();
		}

		SwerveModulePosition[] moduleDeltas = calculateModuleDeltas(getModulePositions());

		// Update gyro angle
		if (gyroIOInfo.isConnected()) {
			// Use the real gyro angle
			rawGyroRotation = gyroIOInfo.getYawPosition();
		} else {
			// Use the angle delta from the kinematics and module deltas
			Twist2d twist = DriveConstants.DRIVE_KINEMATICS.toTwist2d(moduleDeltas);
			rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
		}

		lastModulePositions = getModulePositions();

		Logger.processInputs("Drive/Gyro", gyroIOInfo);
		SmartDashboard.putString("Drive State", getCurrentState().toString());
		odometry.update(rawGyroRotation, getModulePositions());

		switch (currentState) {
			case TELEOP_STATE:
				if (input != null) {
					drive(-input.getControllerLeftJoystickY(),
						-input.getControllerLeftJoystickX(),
						-input.getControllerRightJoystickX(), true);

					if (input.isBackButtonPressed()) {
						gyroIO.resetHeading();
					}
				}
				break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		currentState = nextState(input);
	}

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		switch (currentState) {
			case TELEOP_STATE:
				return FSMState.TELEOP_STATE;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 */
	public void drive(double xSpeed, double ySpeed, double rot,
		boolean fieldRelative) {

		SmartDashboard.putNumber("X Speed", xSpeed);
		SmartDashboard.putNumber("Y Speed", ySpeed);
		SmartDashboard.putNumber("Rot", rot);

		//convert XY to polar and square the magnitude and angle (carrying sign) first
		Rotation2d inputTranslationDir = new Rotation2d(xSpeed, ySpeed);
		double inputTranslationMag = MathUtil.applyDeadband(
				Math.hypot(xSpeed, ySpeed), OIConstants.DRIVE_DEADBAND
		);
		double theta = MathUtil.applyDeadband(rot, OIConstants.DRIVE_DEADBAND);

		inputTranslationMag = inputTranslationMag * inputTranslationMag;
		theta = Math.copySign(theta * theta, theta);

		Translation2d inputVelocity =
				new Pose2d(
					new Translation2d(), inputTranslationDir
				).transformBy(
					new Transform2d(inputTranslationMag, 0.0, new Rotation2d())
				).getTranslation();

		// Convert the commanded speeds into the correct units for the drivetrain
		double xSpeedDelivered = inputVelocity.getX() * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
		double ySpeedDelivered = inputVelocity.getY() * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
		double rotDelivered = theta * DriveConstants.MAX_ANGULAR_SPEED;

		SmartDashboard.putNumber("X Speed Delivered", xSpeedDelivered);
		SmartDashboard.putNumber("Y Speed Delivered", ySpeedDelivered);
		SmartDashboard.putNumber("Rot Speed Delivered", rotDelivered);

		//should run closed loop drive and turn voltage controls based on chassis speeds
		runVelocity(
			fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered,
					rotDelivered, getRotation())
				: new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
		);
	}

	/**
	 * Sets the wheels into an X formation to prevent movement.
	 */
	public void setX() {
		flModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(
			Math.toDegrees(Math.PI / 2))));
		frModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(
			-Math.toDegrees(Math.PI / 2))));
		blModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(
			-Math.toDegrees(Math.PI / 2))));
		brModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(
			Math.toDegrees(Math.PI / 2))));
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	@AutoLogOutput(key = "Odometry/Robot_Pose")
	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	/**
	 * Returns the current odometry rotation.
	 *
	 * @return The pose.
	 */
	@AutoLogOutput(key = "Odometry/Robot_Rotation")
	public Rotation2d getRotation() {
		return getPose().getRotation();
	}

	/**
	 * Returns the module states of all the FSM's ModuleIOs.
	 *
	 * @return List of module states.
	 */
	@AutoLogOutput(key = "SwerveStates/Module_States")
	public SwerveModuleState[] getModuleStates() {
		return new SwerveModuleState[] {
				flModule.getState(),
				frModule.getState(),
				blModule.getState(),
				brModule.getState()
		};
	}

	/**
	 * Returns the module's SwerveModulePositions.
	 *
	 * @return List of module positions.
	 */
	@AutoLogOutput(key = "SwerveStates/Module_Positions")
	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {
				flModule.getPosition(),
				frModule.getPosition(),
				blModule.getPosition(),
				brModule.getPosition()
		};
	}

	/**
	 * Retrives a set of ChassisSpeeds that are robot relative based on the current module states.
	 * @return Robot relative chassis speeds.
	 */
	public ChassisSpeeds getRobotRelativeSpeeds() {
		return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
	}


	/**
	 * Retrieves a set of ChassisSpeeds that are field relative
	 * based on robot relative speeds and rotation.
	 *
	 * @return Field relative chassis speeds.
	 */
	public ChassisSpeeds getFieldRelativeSpeeds() {
		return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getRotation());
	}

	/**
	 * Robot relative driving method used for autos in swerve.
	 * @param robotRelSpeeds The relative chassis speeds.
	 */
	public void runVelocity(ChassisSpeeds robotRelSpeeds) {
		SwerveModuleState[] setpointStates =
			DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(robotRelSpeeds);
		SwerveModuleState[] optimizedSetpointStates = setModuleStates(setpointStates);

		Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
		Logger.recordOutput("SwerveStates/Setpoints_Optimized", optimizedSetpointStates);
	}

	/**
	 * Resets the current odometry pose.
	 * @param pose
	 */
	public void setPose(Pose2d pose) {
		odometry.resetPosition(rawGyroRotation, getModulePositions(), pose);
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param setpointStates The desired SwerveModule states.
	 * @return The optimized setpoint states (mainly for logging)
	 */
	public SwerveModuleState[] setModuleStates(SwerveModuleState[] setpointStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
			setpointStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

		SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[] {
				flModule.setDesiredState(setpointStates[0]),
				frModule.setDesiredState(setpointStates[1]),
				blModule.setDesiredState(setpointStates[2]),
				brModule.setDesiredState(setpointStates[(2 + 1)]),
		};

		return optimizedSetpointStates;
	}

	/** Resets the drive encoders to currently read a position of 0. */
	public void resetEncoders() {
		flModule.resetEncoders();
		frModule.resetEncoders();
		blModule.resetEncoders();
		brModule.resetEncoders();
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from 0 to 360
	 */
	public double getHeading() {
		return gyroIOInfo.getYawPosition().getDegrees();
	}

	private SwerveModulePosition[] calculateModuleDeltas(SwerveModulePosition[] modulePositions) {
		SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[(2 + 2)];
		for (int i = 0; i < (2 + 2); i++) {
			moduleDeltas[i] =
				new SwerveModulePosition(
					modulePositions[i].distanceMeters
							- lastModulePositions[i].distanceMeters,
					modulePositions[i].angle
				);
		}

		return moduleDeltas;
	}

	/**
	 * Returns a command to run a quasistatic test in the specified direction.
	 * @param direction the direction of the SysID routine
	 * @return the sysID command protcol for quasistatic tuning
	*/
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysId.quasistatic(direction);
	}

	/**
	 * Returns a command to run a dynamic test in the specified direction.
	 * @param direction the direction of the SysID routine
	 * @return the sysID command protcol for dynamic tuning
	*/
	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysId.dynamic(direction);
	}
}
