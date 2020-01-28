package org.team2168;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around
 */
public class RobotMap {

    public static final double MAIN_PERIOD_S = 1.0/50.0; // Main loop 200Hz

	/*************************************************************************
	 *                        ROBORIO WIRING MAP                             *
	 *************************************************************************/

	// Joysticks///////////////////////////////////////////////////////////////
	public static final int DRIVER_JOYSTICK = 0;
	public static final int OPERATOR_JOYSTICK = 1;
	public static final int DRIVER_OPERATOR_E_BACKUP = 4;
	public static final int PID_TEST_JOYSTICK = 5;

	// Joystick Control Styles/////////////////////////////////////////////////
	public static final int TANK_DRIVE_STYLE_ENUM = 0;
	public static final int GUN_STYLE_ENUM = 1;
	public static final int ARCADE_STYLE_ENUM = 2;
	public static final int GTA_STYLE_ENUM = 3;
	public static final int NEW_GUN_STYLE_ENUM = 4;

	// PWM (0 to 9) on RoboRio/////////////////////////////////////////////////


	// Digital IO Channels//////////////////////////////////////////////////////
	// Channels 0-9 on RoboRio

	//Channels 10-25 on MXP (PWM and DIO)

	//Analog Input Channels////////////////////////////////////////////////////
	//Channels 0-3 on Roborio

	// Channels 4-7 on MXP
	
	

	/*************************************************************************
	*                         CAN DEVICES                                    *
	*************************************************************************/
	
	//CAN Device IDs///////////////////////////////////////////////////////////
    public static final int PDP_CAN_ID = 0;

	/*************************************************************************
	*                         SPI DEVICES                                    *
	*************************************************************************/

	/*************************************************************************
	*                         Solenoids                                      *
	*************************************************************************/

	//Double Soldenoids PCM ID = 0 ///////////////////////////////////////////
	
	//Double Soldenoids PCM ID = 1 ///////////////////////////////////////////

	/*************************************************************************
	*                         PDP/CAN DEVICES                                 *
    *************************************************************************/
	public static final int DRIVETRAIN_LEFT_MOTOR_1_PDP = 0;
	public static final int DRIVETRAIN_LEFT_MOTOR_2_PDP = 1;
	public static final int DRIVETRAIN_LEFT_MOTOR_3_PDP = 2;
	public static final int DRIVETRAIN_RIGHT_MOTOR_3_PDP = 13;
	public static final int DRIVETRAIN_RIGHT_MOTOR_2_PDP = 14;
	public static final int DRIVETRAIN_RIGHT_MOTOR_1_PDP = 15;
	public static final int DRIVETRAIN_PIGEON_CAN_ID = 17;

	// Relay Channels///////////////////////////////////////////////////////////

	/*************************************************************************
	 *                         PBOT DIFFERENCES  PARAMETERS                  *
	 *************************************************************************/

	

	

	/******************************************************************
	 *                         Lights I2C                             *
	 ******************************************************************/
	// public static final I2C.Port I2C_PORT = I2C.Port.kOnboard;
	// public static final int I2C_ADDRESS = 8;
	// public static final boolean LEDS_REVERSE = true; //true if 0 is at the top
	// public static final boolean LEDS_VERTICAL = true;

}