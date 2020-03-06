
package org.team2168;

import org.team2168.commands.auto.FinishFiring;
import org.team2168.commands.auto.FireBalls;
import org.team2168.commands.climber.Climb;
import org.team2168.commands.climber.DisengageRatchet;
import org.team2168.commands.climber.DriveClimberWithTestJoystickUnSafe;
import org.team2168.commands.climber.DriveClimberXPosition;
import org.team2168.commands.climber.EngageRatchet;
import org.team2168.commands.climber.PrepareToClimb;
import org.team2168.commands.climber.ResetClimberPosition;
import org.team2168.commands.color_wheel.DriveColorWheelXRotations;
import org.team2168.commands.color_wheel_pivot.DisengageColorWheel;
import org.team2168.commands.color_wheel_pivot.EngageColorWheel;
import org.team2168.commands.drivetrain.DriveWithJoystick;
import org.team2168.commands.drivetrain.PIDCommands.LimelightTurnTeleop;
import org.team2168.commands.hood_adjust.MoveToBackTrench;
import org.team2168.commands.hood_adjust.MoveToFiringLocation;
import org.team2168.commands.hood_adjust.MoveToFrontTrench;
import org.team2168.commands.hood_adjust.MoveToWLNoShoot;
import org.team2168.commands.hood_adjust.MoveToWall;
import org.team2168.commands.hood_adjust.MoveToWallNoShoot;
import org.team2168.commands.hood_adjust.MoveToWhiteLine;
import org.team2168.commands.hopper.DriveHopperWithConstant;
import org.team2168.commands.indexer.DriveIndexerWithConstant;
import org.team2168.commands.intakeMotor.DriveIntakeWithConstant;
import org.team2168.commands.intakeMotor.IntakeBallStart;
import org.team2168.commands.intakeMotor.IntakeBallStop;
import org.team2168.commands.intakePivot.ExtendIntakePneumatic;
import org.team2168.commands.intakePivot.RetractIntakePneumatic;
import org.team2168.commands.shooter.DriveShooterWithConstant;
import org.team2168.commands.shooter.DriveToXSpeed;
import org.team2168.subsystems.Shooter;
import org.team2168.utils.F310;
import org.team2168.utils.LinearInterpolator;


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
	public F310 buttonBox1 = new F310(RobotMap.BUTTON_BOX_1);
	public F310 buttonBox2 = new F310(RobotMap.BUTTON_BOX_2);

	// public F310 driverOperatorEJoystick = new
	// F310(RobotMap.DRIVER_OPERATOR_E_BACKUP);

	// public F310 testJoystick = new F310(RobotMap.COMMANDS_TEST_JOYSTICK);
	//public F310 pidTestJoystick = new F310(RobotMap.PID_TEST_JOYSTICK);
	private LinearInterpolator gunStyleYInterpolator;
	private LinearInterpolator gunStyleXInterpolator;
	private LinearInterpolator colorWheelInterpolator;
	private LinearInterpolator climberInterpolator;
	private LinearInterpolator balancerInterpolator;
	private LinearInterpolator climberResetInterpolator;

	private double[][] gunStyleYArray = {
		{-1.0, -1.00}, //can limit speed by changing second number
		{-0.15, 0.00},
		{+0.15, 0.00},
		{+1.00, +1.00}
	};
	private double[][] gunStyleXArray = {
		{-1.0, -0.70},  //scale down turning to max 70%
		{-0.05, 0.00},  //set neutral deadband to 5%
		{+0.05, 0.00},
		{+1.00,+0.70}  
	};

	private double[][] colorWheelArray = {
		{-1.0, -1.00},  
		{-0.05, 0.00},  //set neutral deadband to 4%
		{+0.05, 0.00},
		{+1.0, +1.00}  
	};

	private double[][] climberArray = {
		{-1.0, -1.00},  
		{-0.50, 0.00},  //set neutral deadband to 4%
		{+0.50, 0.00},
		{+1.0, +1.00}  
	};

	private double[][] balancerArray = {
		{-1.0, -1.00},  
		{-0.00, 0.00},  //set neutral deadband to 4%
		{+0.00, 0.00},
		{+1.0, +1.00}  
	};

	private double[][] climberResetArray = {
		{-1.0, -0.15},  
		{-0.05, 0.00},
		{+0.05, 0.00},
		{+1.0, +0.15}  
	};

	/**
	 * Private constructor for singleton class which instantiates the OI object
	 */
	private OI() {

		if (Robot.ENABLE_BUTTON_BOX)
	{
		//******************************************************************* */
		//*							Button Box I
		//******************************************************************* */
		colorWheelInterpolator = new LinearInterpolator(colorWheelArray);


		buttonBox1.ButtonUpDPad().whenPressed(new EngageColorWheel());
		buttonBox1.ButtonDownDPad().whenPressed(new DisengageColorWheel());
		buttonBox1.ButtonLeftDPad().whenPressed(new DriveColorWheelXRotations(-4.0*8.0)); 	//go opposite direction to protect limelight
		//Right D Pad, Position
		// Button A, bump up = increment velocity adjustment of shooter
		// Button B, bump down
		// Button X, reset
		buttonBox1.ButtonY().whenPressed(new DriveToXSpeed(Shooter.FiringLocation.BACK_TRENCH));
		buttonBox1.ButtonLeftBumper().whenPressed(new DriveHopperWithConstant(-1.0));// Temporary value
		buttonBox1.ButtonLeftBumper().whenPressed(new DriveIndexerWithConstant(-1.0));
		buttonBox1.ButtonLeftBumper().whenReleased(new DriveHopperWithConstant(0.0));
		buttonBox1.ButtonLeftBumper().whenReleased(new DriveIndexerWithConstant(0.0));
		buttonBox1.ButtonRightBumper().whenPressed(new MoveToWallNoShoot());
		buttonBox1.ButtonRightBumper().whenPressed(new DisengageColorWheel());

		//******************************************************************** */
		//*							Button Box II
		//******************************************************************** */
		climberInterpolator = new LinearInterpolator(climberArray);
		balancerInterpolator = new LinearInterpolator(balancerArray);

		buttonBox2.ButtonDownDPad().whenPressed(new DriveToXSpeed(Shooter.FiringLocation.WALL));
		buttonBox2.ButtonLeftDPad().whenPressed(new DriveToXSpeed(Shooter.FiringLocation.FRONT_TRENCH));
		buttonBox2.ButtonRightDPad().whenPressed(new DriveToXSpeed(Shooter.FiringLocation.WHITE_LINE));
		// buttonBox2.ButtonA().whenPressed(new MoveToFiringLocation(Shooter.getInstance().getFiringLocation()));
		// pidTestJoystick.ButtonA().whenReleased(new MoveToWallNoShoot());

		buttonBox2.ButtonB().whenPressed(new FireBalls());
		buttonBox2.ButtonB().whenReleased(new FinishFiring());

		//spit button
		buttonBox2.ButtonLeftBumper().whenPressed(new DriveIntakeWithConstant(-1.0));
		buttonBox2.ButtonLeftBumper().whenPressed(new ExtendIntakePneumatic());
		buttonBox2.ButtonLeftBumper().whenReleased(new DriveIntakeWithConstant(0.0));
		buttonBox2.ButtonLeftBumper().whenReleased(new RetractIntakePneumatic()); //DO WE WANT THIS

		buttonBox2.ButtonRightBumper().whenPressed(new IntakeBallStart());
		buttonBox2.ButtonRightBumper().whenReleased(new IntakeBallStop());

		buttonBox2.ButtonBack().whenPressed(new PrepareToClimb());
		buttonBox2.ButtonBack().whenPressed(new DriveToXSpeed(0.0)); //Stop shooter and lower hood
		buttonBox2.ButtonBack().whenPressed(new MoveToWLNoShoot());
		buttonBox2.ButtonStart().whenPressed(new Climb());
		//right stick--auto balance
	}
		/*************************************************************************
		 * Driver Joystick *
		 *************************************************************************/
		gunStyleYInterpolator = new LinearInterpolator(gunStyleYArray);
		gunStyleXInterpolator = new LinearInterpolator(gunStyleXArray);

		driverJoystick.ButtonLeftStick().whileHeld(new LimelightTurnTeleop(1.50));
		driverJoystick.ButtonLeftStick().whenReleased(new DriveWithJoystick());

		// driverJoystick.ButtonLeftStick().whenPressed(new RunFlashlight(1.0));
		// driverJoystick.ButtonLeftStick().whenReleased(new RunFlashlight(0.0));

		//When the red button on the handle of the controller is pressed get ready to go under the trechn. Lower everything.
		driverJoystick.ButtonLeftBumper().whileHeld(new DisengageColorWheel());
		driverJoystick.ButtonLeftBumper().whenPressed(new MoveToFiringLocation(Shooter.FiringLocation.WALL));
		
		/*************************************************************************
		 * Operator Joystick *
		 *************************************************************************/
		operatorJoystick.ButtonUpDPad().whenPressed(new MoveToBackTrench());
		operatorJoystick.ButtonLeftDPad().whenPressed(new MoveToFrontTrench());
		operatorJoystick.ButtonRightDPad().whenPressed(new MoveToWhiteLine());
		operatorJoystick.ButtonDownDPad().whenPressed(new MoveToWall());


		operatorJoystick.ButtonY().whenPressed(new EngageColorWheel());
		operatorJoystick.ButtonA().whenPressed(new DisengageColorWheel());
		// operatorJoystick.ButtonA().whenPressed(new MoveToWall());

		operatorJoystick.ButtonStart().whenPressed(new DriveColorWheelXRotations(-4.0*8.0)); 	//go opposite direction to protect limelight 


		// operatorJoystick.ButtonX().whenPressed(new DriveToXSpeed(Shooter.getInstance().WALL_VEL));
		operatorJoystick.ButtonX().whenPressed(new DriveShooterWithConstant(0.52));

		operatorJoystick.ButtonB().whenPressed(new FireBalls());
		operatorJoystick.ButtonB().whenReleased(new FinishFiring());

		operatorJoystick.ButtonLeftBumper().whenPressed(new IntakeBallStop());
		operatorJoystick.ButtonRightBumper().whenPressed(new IntakeBallStart());




		/***********************************************************************
		 * Commands Test Joystick
		 ***********************************************************************/
		// // //leds testing
		// pidTestJoystick.ButtonB().whenPressed(new DriveXDistance(60.0));
		// pidTestJoystick.ButtonX().whenPressed(new DriveXDistance(-60.0));
		// pidTestJoystick.ButtonRightDPad().whenPressed(new TurnXAngle(-9.0, 0.3));
		// pidTestJoystick.ButtonLeftDPad().whenPressed(new TurnXAngle(+9.0,0.3));
		// pidTestJoystick.ButtonUpDPad().whenPressed(new TurnXAngle(-90.0,0.3));
		// pidTestJoystick.ButtonDownDPad().whenPressed(new TurnXAngle(+90.0, 0.3));
		// pidTestJoystick.ButtonStart().whenPressed(new DefaultTrenchAuto());
		// pidTestJoystick.ButtonBack().whenPressed(new OppositeTrenchAuto());

		pidTestJoystick.ButtonX().whenPressed(new ResetClimberPosition());
		pidTestJoystick.ButtonY().whenPressed(new PrepareToClimb()); 
		pidTestJoystick.ButtonA().whenPressed(new Climb());

		pidTestJoystick.ButtonUpDPad().whenPressed(new EngageRatchet());
		pidTestJoystick.ButtonDownDPad().whenPressed(new DisengageRatchet());

		climberResetInterpolator = new LinearInterpolator(climberResetArray);
		pidTestJoystick.ButtonRightStick().whenPressed(new DriveClimberWithTestJoystickUnSafe());

		// pidTestJoystick.ButtonB().whenPressed(new FireBalls());
		// pidTestJoystick.ButtonB().whenReleased(new FinishFiring());
		// pidTestJoystick.ButtonDownDPad().whenPressed(new MoveToWall());
		// pidTestJoystick.ButtonLeftBumper().whenPressed(new IntakeBallStop());
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
		return gunStyleXInterpolator.interpolate(driverJoystick.getLeftStickRaw_X());
	}

	public double getGunStyleYValue()
	{
		return gunStyleYInterpolator.interpolate(driverJoystick.getLeftStickRaw_Y());
	}

	/**
	 * Method that sets that Left side of the drive train so that it drives with
	 * LeftStick Y
	 * 
	 */
	public double getDriveTrainLeftJoystick()
	{
		return driverJoystick.getLeftStickRaw_Y();
	}

	/**
	 * Method that sets that Right side of the drive train so that it drives with
	 * RightStick Y
	 *
	 */
	
	public double getDriveTrainRightJoystick()
	{
		return driverJoystick.getRightStickRaw_Y();
	}

	public double getColorWheelJoystick()
	{
		return colorWheelInterpolator.interpolate(buttonBox1.getLeftStickRaw_X()); //operatorJoystick.getRightStickRaw_Y();
	}

	public double getIntakeMotorJoyStick()
	{
		return operatorJoystick.getRightTriggerAxisRaw() - operatorJoystick.getLeftTriggerAxisRaw();
	}

	/**
	 * Return value of axis for the indexer
	 * 
	 * @return a double - value
	 */
	public double getIndexerJoystick()
	{
		return 0.0; //pidTestJoystick.getRightStickRaw_Y();
	}

	/**
	 * Climber joystick value
	 * 
	 * @return
	 */
	public double getClimberJoystickValue()
	{
		return climberInterpolator.interpolate(buttonBox2.getLeftStickRaw_Y()); 
	}

	public double getClimberTestJoystickValue() {
		return climberResetInterpolator.interpolate(pidTestJoystick.getRightStickRaw_Y());
	}

	/**
	 * Shooter joystick value
	 * @return
	 */
	public double getShooterJoystick()
	{
		return 0.0; //pidTestJoystick.getRightStickRaw_Y();
	}

	/**
	 * Balancer joystick value
	 * 
	 * @return
	 */
	public double getBalancerJoystickValue()
	{
		return  balancerInterpolator.interpolate(buttonBox2.getLeftStickRaw_X()); //pidTestJoystick.getLeftStickRaw_Y();
	}

	/**
	 * Hopper joystick value
	 * 
	 * @return
	 */
	public double getHopperJoystickValue()
	{
		return  0.0; //pidTestJoystick.getRightStickRaw_Y();
	}

	
}