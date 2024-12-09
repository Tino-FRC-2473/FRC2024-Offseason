package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.PS4Controller;

/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/* ======================== Constants ======================== */

	/* ======================== Private variables ======================== */
	// Input objects
	private PS4Controller ps4Controller;
	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		ps4Controller = new PS4Controller(0);
	}


	/* ------------------------ Mech Controller ------------------------ */

	/**
	 * Get the value of the Square Button.
	 * @return if Square Button is pressed
	 */
	public boolean isPauseButtonPressed() {
		return ps4Controller.getSquareButtonPressed();
	}

	/**
	 * Get the value of the Circle Button.
	 * @return if Circle Button is pressed
	 */
	public boolean isStopButtonPressed() {
		return ps4Controller.getCircleButtonPressed();
	}

	/**
	 * Get the value of the Triangle Button.
	 * @return if Triangle Button is pressed
	 */
	public boolean isPlayButtonPressed() {
		return ps4Controller.getTriangleButtonPressed();
	}
}
