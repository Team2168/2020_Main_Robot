package org.team2168; 


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around\][\ */
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

	//Double Soldenoids PCM ID = 0 ///////////////////////////////////////////
	public static final int BALANCER_ENGAGE_PCM = 0
	public static final int BALANCE_DISENGAGE_PCM = 1
	
	//Double Soldenoids PCM ID = 1 ///////////////////////////////////////////


	/*************************************************************************
	*                         PDP/CAN DEVICES                                 *
    *************************************************************************/
    
	// Relay Channels///////////////////////////////////////////////////////////

	/*************************************************************************
	 *                         PBOT DIFFERENCES  PARAMETERS                  *
	 *************************************************************************/

	/*************************************************************************
	 *                         DRIVETRAIN PARAMETERS                         *
	 *************************************************************************/
	// TODO check if the reverse values match the physical robot
    
	// public static final boolean DT_3_MOTORS_PER_SIDE = true;

	// private static final int DRIVE_PULSE_PER_ROTATION = 256; // encoder ticks per rotation

	// private static final double DRIVE_GEAR_RATIO = 2.0 / 1.0; // ratio between wheel over encoder
	// private static final double DRIVE_WHEEL_DIAMETER = 5.0;   //inches;
	// public static final int DRIVE_ENCODER_PULSE_PER_ROT = (int) (DRIVE_PULSE_PER_ROTATION * DRIVE_GEAR_RATIO); // pulse per rotation * gear																					// ratio
	
	// public static final double DRIVE_ENCODER_DIST_PER_TICK = (Math.PI * DRIVE_WHEEL_DIAMETER / DRIVE_ENCODER_PULSE_PER_ROT);
	// public static final CounterBase.EncodingType DRIVE_ENCODING_TYPE = CounterBase.EncodingType.k4X; // count rising and falling edges on
	// public static final AverageEncoder.PositionReturnType DRIVE_POS_RETURN_TYPE = AverageEncoder.PositionReturnType.INCH;
	// public static final AverageEncoder.SpeedReturnType DRIVE_SPEED_RETURN_TYPE = AverageEncoder.SpeedReturnType.IPS;
	// public static final int DRIVE_ENCODER_MIN_RATE = 0;
	// public static final int DRIVE_ENCODER_MIN_PERIOD = 1;
	// public static final boolean LEFT_DRIVE_TRAIN_ENCODER_REVERSE = true;
	// public static final boolean RIGHT_DRIVE_TRAIN_ENCODER_REVERSE = true;

	// public static final int DRIVE_AVG_ENCODER_VAL = 5;
	// public static final double MIN_DRIVE_SPEED = 0.2;
	// public static final double AUTO_NORMAL_SPEED = 0.5;
	// public static final double WHEEL_BASE = 26; //units must match PositionReturnType (inch)

	/*************************************************************************
	 *                         PID PARAMETERS                                *
	 *************************************************************************/
	// period to run PID loops on drive train
	// public static final long DRIVE_TRAIN_PID_PERIOD = 20;// 70ms loop
	// public static final int DRIVE_TRAIN_PID_ARRAY_SIZE = 30;

	// public static final double DRIVE_TRAIN_MIN_FWD_VOLTAGE = 1.8;//volts
	// public static final double DRIVE_TRAIN_MIN_RVD_VOLTAGE = 1.2;//volts

	// public static final double DRIVE_TRAIN_MIN_ROT_CLOCKWISE_VOLTAGE = 1.45;//volts
	// public static final double DRIVE_TRAIN_MIN_ROT_COUNTCLOCKWISE_VOLTAGE = 1.45;//volts

	// PID Gains for Left & Right Speed and Position
	// Bandwidth =
	// Phase Margin =
	// public static final double DRIVE_TRAIN_LEFT_SPEED_P = 0.04779;
	// public static final double DRIVE_TRAIN_LEFT_SPEED_I = 0.0010526;
	// public static final double DRIVE_TRAIN_LEFT_SPEED_D = 0.0543;

	// public static final double DRIVE_TRAIN_RIGHT_SPEED_P = 0.04779;
	// public static final double DRIVE_TRAIN_RIGHT_SPEED_I = 0.0010526;
	// public static final double DRIVE_TRAIN_RIGHT_SPEED_D = 0.0543;

	// public static final double DRIVE_TRAIN_LEFT_POSITION_P = 0.2;
	// public static final double DRIVE_TRAIN_LEFT_POSITION_I = 0.0001412646174233;
	// public static final double DRIVE_TRAIN_LEFT_POSITION_D = 0.0074778888124088;

	// public static final double DRIVE_TRAIN_RIGHT_POSITION_P = 0.25;
	// public static final double DRIVE_TRAIN_RIGHT_POSITION_I = 0.0001412646174233;
	// public static final double DRIVE_TRAIN_RIGHT_POSITION_D = 0.0074778888124088;


	// public static final double ROTATE_POSITION_P = 0.055;
	// public static final double ROTATE_POSITION_I = 0.001;
	// public static final double ROTATE_POSITION_D = 0.0064778888124088;

	// public static final double ROTATE_POSITION_P_Drive_Straight = 0.055; //0.055 comp
	// public static final double ROTATE_POSITION_I_Drive_Straight = 0.001; //0.001
	// public static final double ROTATE_POSITION_D_Drive_Straight = 0.0064778888124088;


	// public static final double LIFT_P = 0.044;
	// public static final double LIFT_I = 0.0020;
	// public static final double LIFT_D = 0.0001;

	// public static final double LIMELIGHT_POSITION_P = 0.013;
	// public static final double LIMELIGHT_POSITION_I = 0.0;
	// public static final double LIMELIGHT_POSITION_D = 0.0;
		
	// public static final long LIFT_PID_PERIOD = 20;
	// public static final int  LIFT_PID_ARRAY_SIZE = 30;

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
	
}