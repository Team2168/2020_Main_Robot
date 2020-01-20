
package org.team2168;

import org.team2168.utils.F310;
import org.team2168.utils.LinearInterpolator;
import org.team2168.commands.drivetrain.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI
{
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
	private static OI instance = null;

	public F310 driverJoystick = new F310(RobotMap.DRIVER_JOYSTICK);
	public F310 operatorJoystick = new F310(RobotMap.OPERATOR_JOYSTICK);
	public F310 pidTestJoystick = new F310(RobotMap.PID_TEST_JOYSTICK);

	// public F310 driverOperatorEJoystick = new
	// F310(RobotMap.DRIVER_OPERATOR_E_BACKUP);

	// public F310 testJoystick = new F310(RobotMap.COMMANDS_TEST_JOYSTICK);
	//public F310 pidTestJoystick = new F310(RobotMap.PID_TEST_JOYSTICK);
	private LinearInterpolator gunStyleInterpolator;
	private double[][] gunStyleArray = {
		{ -1.0, -1.0},
		{ -.15, 0.0},
		{ .15, 0.0},
		{ 1.0, 1.0}
	};

	/**
	 * Private constructor for singleton class which instantiates the OI object
	 */
	private OI() {

		/*************************************************************************
		 * Driver Joystick *
		 *************************************************************************/
		
		gunStyleInterpolator = new LinearInterpolator(gunStyleArray);

		
		/*************************************************************************
		 * Operator Joystick *
		 *************************************************************************/

		/***********************************************************************
		 * Commands Test Joystick
		 ***********************************************************************/
		// //leds testing
		// pidTestJoystick.ButtonA().whenPressed(new DisabledPattern());
		// pidTestJoystick.ButtonB().whenPressed(new TeleopWithoutGamePiecePattern());
		// pidTestJoystick.ButtonX().whenPressed(new AutoWithoutGamePiecePattern());
		// pidTestJoystick.ButtonY().whenPressed(new HABClimbPattern());
		// pidTestJoystick.ButtonLeftBumper().whenPressed(new PivotingPattern());
		// pidTestJoystick.ButtonRightBumper().whenPressed(new LiftRaisingPattern());
		// pidTestJoystick.ButtonRightTrigger().whenPressed(new LiftLoweringPattern());
		// pidTestJoystick.ButtonDownDPad().whenPressed(new WheelsInPattern());
		// pidTestJoystick.ButtonLeftDPad().whenPressed(new WheelsOutPattern());
		// pidTestJoystick.ButtonRightDPad().whenPressed(new MonkeyBarPattern());
		// pidTestJoystick.ButtonUpDPad().whenPressed(new WithGamePiecePattern());

		
	}
	

	/**
	 * Returns an instance of the Operator Interface.
	 * 
	 * @return is the current OI object
	 */
	public static OI getInstance()
	{
		if (instance == null)
			instance = new OI();

		return instance;
	}

	/*************************************************************************
	 * Drivetrain *
	 *************************************************************************/

	public double getGunStyleXValue()
	{
		return driverJoystick.getLeftStickRaw_X();
	}

	public double getGunStyleYValue()
	{

		return gunStyleInterpolator.interpolate(driverJoystick.getLeftStickRaw_Y());
	}

	/**
	 * Method that sets that Left side of the drive train so that it drives with
	 * LeftStick Y
	 * 
	 * @author Krystina
	 */
	public double getDriveTrainLeftJoystick()
	{
		return driverJoystick.getLeftStickRaw_Y();
	}

	/**
	 * Method that sets that Right side of the drive train so that it drives with
	 * RightStick Y
	 * /
	 * @author Krystina
	 */
	public double getDriveTrainRightJoystick()
	{
		return driverJoystick.getRightStickRaw_Y();
	}

}