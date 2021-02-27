/**
 * Simple class containing constants used throughout project
 */
package org.team2168;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class Constants {
	/**
	 * Number of joystick buttons to poll.
	 * 10 means buttons[1,9] are polled, which is actually 9 buttons.
	 */
	public final static int kNumButtonsPlusOne = 10;
	
	/**
	 * How many sensor units per rotation.
	 * Using Talon FX Integrated Encoder.
	 * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
	 */
//	public final static int kSensorUnitsPerRotation = 2048;
	
	/**
	 * Number of rotations to drive when performing Distance Closed Loop
	 */
	public final static double kRotationsToTravel = 5;
	
	/**
	 * This is a property of the Pigeon IMU, and should not be changed.
	 */
	public final static int kPigeonUnitsPerRotation = 8192;

	/**
	 * Set to zero to skip waiting for confirmation.
	 * Set to nonzero to wait and report to DS if action fails.
	 */
	public final static int kTimeoutMs = 30;

	/**
	 * Motor neutral dead-band, set to the minimum 0.1%.
	 */
	public final static double kNeutralDeadband = 0.001;
	
	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
     * Not all set of Gains are used in this project and may be removed as desired.
     * 
	 * 	                                    			   kP     kI     kD   kF             Iz    PeakOut */
	// public final static Gains kGains_Distance = new Gains( 0.125,  0.00,   0.0, 0.0,            120,  0.75 ); //always used for linear path
	// public final static Gains kGains_Turning  = new Gains( 0.48, 0.00,  0.0, 0.0,            200,  0.5 ); //used to turn during autos
	// public final static Gains kGains_Turning_Straight  = new Gains( 3.9, 0.0, 0.0, 0.0,  300,  0.50 ); //used to maintain heading while auto driving straight
	// public final static Gains kGains_Limelight  = new Gains( 0.55, 0.0,  0.0, 1023.0/6800.0,  200,  0.5 );
	
	/** ---- Flat constants, you should not need to change these ---- */
	/* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
	public final static int REMOTE_0 = 0;
	public final static int REMOTE_1 = 1;
	/* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
	public final static int PID_PRIMARY = 0;
	public final static int PID_TURN = 1;
	/* Firmware currently supports slots [0, 3] and can be used for either PID Set */
	public final static int SLOT_0 = 0;
	public final static int SLOT_1 = 1;
	public final static int SLOT_2 = 2;
	public final static int SLOT_3 = 3;
	/* ---- Named slots, used to clarify code ---- */
	public final static int kSlot_Distanc = SLOT_0;
	public final static int kSlot_Turning = SLOT_1;
	public final static int kSlot_Velocit = SLOT_2;
	public final static int kSlot_MotProf = SLOT_3;

	/* ------- New drivetrain gains ------ */
	// TODO all gains are in meters for now, but once I figure out everything I will assess whether this is needed
	public final static double kDriveS = 0.639;
	public final static double kDriveV = 1.99;
	public final static double kDriveA = 0.265;
	// public final static double kDriveP = 1.71; //1.5 m/s acceptable error
	public final static double kDriveP = 0.0; // 0.2 m/s acceptable error
	// public static final double kDriveP = 0.0;
	public final static double kDriveI = 0.0;
	public final static double kDriveD = 0.0;

	public final static double kTrackWidthMeters = 1.480054863949271;
	public static final DifferentialDriveKinematics kDriveKinematics = 
		new DifferentialDriveKinematics(kTrackWidthMeters);

	public static final double kMaxSpeedMetersPerSecond = 0.01;
	public static final double kMaxAccelerationMetersPerSecondSquared = 0.01;
	
		

	// Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}