package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/* ======================== Constants ======================== */
	private static final int MECH_CONTROLLER_PORT = 1;

	/* ======================== Private variables ======================== */
	// Input objects
	private PS4Controller ps4Controller;
	private XboxController xboxController;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		ps4Controller = new PS4Controller(0);
		// xboxController = new XboxController(MECH_CONTROLLER_PORT);
	}


	/* ------------------------ Mech Controller ------------------------ */

	/**
	 * Get the value of the Circle Button.
	 * @return if Circle Button is pressed
	 */
	public boolean isShooterButtonPressed() {
		return ps4Controller.getCircleButton();
		// return xboxController.getXButton();
	}
}
