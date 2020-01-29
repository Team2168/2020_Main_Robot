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
    public static final int PDP_CAN_ID = 0;

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
	
	//Double Soldenoids PCM ID = 1 ///////////////////////////////////////////


	/*************************************************************************
	*                         PDP/CAN DEVICES                                 *
    *************************************************************************/
	public static final int CLIMBER_MOTOR_1_PDP = 3;
	public static final int CLIMBER_MOTOR_2_PDP = 12;
    public static final int BALANCER_MOTOR_PDP = 7;
	public static final int DRIVETRAIN_LEFT_MOTOR_1_PDP = 0;
	public static final int DRIVETRAIN_LEFT_MOTOR_2_PDP = 1;
	public static final int DRIVETRAIN_LEFT_MOTOR_3_PDP = 2;
	public static final int DRIVETRAIN_RIGHT_MOTOR_1_PDP = 15;
	public static final int DRIVETRAIN_RIGHT_MOTOR_2_PDP = 14;
	public static final int DRIVETRAIN_RIGHT_MOTOR_3_PDP = 13;

	// Relay Channels///////////////////////////////////////////////////////////

	/*************************************************************************
	 *                         PBOT DIFFERENCES  PARAMETERS                  *
	 *************************************************************************/

	/*************************************************************************
	 *                         DRIVETRAIN PARAMETERS                         *
	 *************************************************************************/
	

	

	/*************************************************************************
	 *                         PID PARAMETERS                                *
	 *************************************************************************/
	// period to run PID loops on drive train
	public static final long DRIVE_TRAIN_PID_PERIOD = 20;// 70ms loop
	public static final int DRIVE_TRAIN_PID_ARRAY_SIZE = 30;

	public static final double DRIVE_TRAIN_MIN_FWD_VOLTAGE = 1.8;//volts
	public static final double DRIVE_TRAIN_MIN_RVD_VOLTAGE = 1.2;//volts

	public static final double DRIVE_TRAIN_MIN_ROT_CLOCKWISE_VOLTAGE = 1.45;//volts
	public static final double DRIVE_TRAIN_MIN_ROT_COUNTCLOCKWISE_VOLTAGE = 1.45;//volts

	// PID Gains for Left & Right Speed and Position
	// Bandwidth =

	public static final double DRIVE_TRAIN_LEFT_SPEED_P = 0.04779;
	public static final double DRIVE_TRAIN_LEFT_SPEED_I = 0.0010526;
	public static final double DRIVE_TRAIN_LEFT_SPEED_D = 0.0543;

	public static final double DRIVE_TRAIN_RIGHT_SPEED_P = 0.04779;
	public static final double DRIVE_TRAIN_RIGHT_SPEED_I = 0.0010526;
	public static final double DRIVE_TRAIN_RIGHT_SPEED_D = 0.0543;

	public static final double DRIVE_TRAIN_LEFT_POSITION_P = 0.2;
	public static final double DRIVE_TRAIN_LEFT_POSITION_I = 0.0001412646174233;
	public static final double DRIVE_TRAIN_LEFT_POSITION_D = 0.0074778888124088;

	public static final double DRIVE_TRAIN_RIGHT_POSITION_P = 0.25;
	public static final double DRIVE_TRAIN_RIGHT_POSITION_I = 0.0001412646174233;
	public static final double DRIVE_TRAIN_RIGHT_POSITION_D = 0.0074778888124088;

	public static final double ROTATE_POSITION_P = 0.055;
	public static final double ROTATE_POSITION_I = 0.001;
	public static final double ROTATE_POSITION_D = 0.0064778888124088;


	public static final double ROTATE_POSITION_P_Drive_Straight = 0.055; //0.055 comp
	public static final double ROTATE_POSITION_I_Drive_Straight = 0.001; //0.001
	public static final double ROTATE_POSITION_D_Drive_Straight = 0.0064778888124088;

	

	public static final double LIFT_P = 0.044;
	public static final double LIFT_I = 0.0020;
	public static final double LIFT_D = 0.0001;

	public static final double LIMELIGHT_POSITION_P = 0.013;
	public static final double LIMELIGHT_POSITION_I = 0.0;
	public static final double LIMELIGHT_POSITION_D = 0.0;
		
	public static final long LIFT_PID_PERIOD = 20;
	public static final int  LIFT_PID_ARRAY_SIZE = 30;



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
	// public static final int TCP_SERVER_ROTATE_CONTROLLER_WITH_CAMERA = 1188;
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
	/******************************************************************
	 *                         PDP PARAMETERS                         *
	 ******************************************************************/
	public static final long PDPThreadPeriod = 100;
}