package org.team2168;

import org.team2168.PID.sensors.AverageEncoder;

import edu.wpi.first.wpilibj.CounterBase;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around\][\
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
	public static final int PCM_CAN_ID_BELLYPAN = 0;
	public static final int PCM_CAN_ID_SHOOTER = 1;

	/*************************************************************************
	*                         SPI DEVICES                                    *
	*************************************************************************/
	public static final int GYRO = 0;


	/*************************************************************************
	*                         Solenoids                                      *
	*************************************************************************/
	public static final int CLIMBER_RATCHET_ENGAGE_PCM = 2;
	public static final int CLIMBER_RATCHET_DISENGAGE_PCM = 3;

	//Double Soldenoids PCM ID = 0 ///////////////////////////////////////////
	public static final int INTAKE_ENGAGE_PCM = 0;
	public static final int INTAKE_DISENGAGE_PCM = 1;
	public static final int COLORWHEEL_ENGAGE_PCM = 4;
	public static final int COLORWHEEL_DISENGAGE_PCM = 5;
	//Double Soldenoids PCM ID = 1 ///////////////////////////////////////////

	public static final int PANCAKE_SOLENOID_IN = 1;
	public static final int PANCAKE_SOLENOID_OUT = 0;
	public static final int HOOD_SOLENOID_ENGAGE = 2;
	public static final int HOOD_SOLENOID_DISENGAGE = 3;
	

	/*************************************************************************
	*                         PDP/CAN DEVICES                                 *
    *************************************************************************/
	public static final int DRIVETRAIN_LEFT_MOTOR_1_PDP = 0;
	public static final int DRIVETRAIN_LEFT_MOTOR_2_PDP = 1;
	public static final int DRIVETRAIN_LEFT_MOTOR_3_PDP = 2;
	public static final int DRIVETRAIN_RIGHT_MOTOR_1_PDP = 15;
	public static final int DRIVETRAIN_RIGHT_MOTOR_2_PDP = 14;
	public static final int DRIVETRAIN_RIGHT_MOTOR_3_PDP = 13;
	public static final int CLIMBER_MOTOR_1_PDP = 3;
	public static final int CLIMBER_MOTOR_2_PDP = 12;
	public static final int INTAKE_MOTOR_PDP = 5;
	public static final int INDEXER_MOTOR_PDP = 6;
	public static final int BALANCER_MOTOR_PDP = 7;
	public static final int HOPPER_MOTOR_PDP = 9;
	public static final int COLORWHEEL_MOTOR_PDP = 10;


	// Relay Channels///////////////////////////////////////////////////////////
	public static final int SHOOTER_MOTOR_ONE_PDP = 4;
	public static final int SHOOTER_MOTOR_TWO_PDP = 11;

	/*************************************************************************
	 *                         PBOT DIFFERENCES  PARAMETERS                  *
	 *************************************************************************/

	/*************************************************************************
	 *                         DRIVETRAIN PARAMETERS                         *
	 *************************************************************************/
	

	

	/*************************************************************************
	 *                         PDP PARAMETERS                                *
	 *************************************************************************/

	/*************************************************************************
	 *                         PID PARAMETERS                                *
	 *************************************************************************/

	/****************************************************************
	 *                         TCP Servers (ONLY FOR DEBUGGING)     *
	 ****************************************************************/
	// public static final int TCP_SERVER_DRIVE_TRAIN_POS = 1180;
	// public static final int TCP_SERVER_ROTATE_CONTROLLER = 1181;
	// public static final int TCO_SERVER_RIGHT_DRIVE_TRAIN_SPEED = 1182;
	// public static final int TCP_SERVER_LEFT_DRIVE_TRAIN_SPEED = 1183;
	// public static final int TCP_SERVER_LIFT_POT_CONTROLLER = 1184;
	// public static final int TCP_SERVER_ROTATE_CONTROLLER_STRAIGHT = 1185;
	// public static final int TCP_SERVER_RIGHT_DRIVE_TRAIN_POSITION = 1186;
	// public static final int TCP_SERVER_LEFT_DRIVE_TRAIN_POSITION = 1187;
	public static final int TCP_SERVER_ROTATE_CONTROLLER_WITH_CAMERA = 1188;
	// public static final int TCP_SERVER_MB_POT_CONTROLLER = 1189;
	// public static final int TCP_SERVER_HP_POT_CONTROLLER = 1190;
	// public static final int TCP_SERVER_RIGHT_STINGER_POSITION = 1191;
	// public static final int TCP_SERVER_LEFT_STINGER_POSITION = 1192;

	

	/******************************************************************
	 *                         ConsolePrinter PARAMETERS              *
	 ******************************************************************/
	// public static final boolean PRINT_SD_DEBUG_DATA = false;
	// public static final long SmartDashThreadPeriod = 200; // ms
	// public static final long CONSOLE_PRINTER_LOG_RATE_MS = 200; // ms

	/******************************************************************
	 *                         Lights I2C                             *
	 ******************************************************************/
	// public static final I2C.Port I2C_PORT = I2C.Port.kOnboard;
	// public static final int I2C_ADDRESS = 8;
	// public static final boolean LEDS_REVERSE = true; //true if 0 is at the top
	// public static final boolean LEDS_VERTICAL = true;
}