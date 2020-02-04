/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /** Hardware */
  private static TalonFX _leftMotor1 = new TalonFX(0);
  private static TalonFX _leftMotor2 = new TalonFX(1);
  private static TalonFX _leftMotor3 = new TalonFX(2);
  private static TalonFX _rightMotor1 = new TalonFX(15);
  private static TalonFX _rightMotor2 = new TalonFX(14);
  private static TalonFX _rightMotor3 = new TalonFX(13);
  private static PigeonIMU _pidgey = new PigeonIMU(17);

  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 40; //amps
  private final double TRIGGER_THRESHOLD_LIMIT = 60; //amp
  private final double TRIGGER_THRESHOLD_TIME = 200; //ms
  public static final boolean DT_REVERSE_LEFT1 = false;
	public static final boolean DT_REVERSE_LEFT2 = false;
	public static final boolean DT_REVERSE_LEFT3 = false;
	public static final boolean DT_REVERSE_RIGHT1 = true;
	public static final boolean DT_REVERSE_RIGHT2 = true;
  public static final boolean DT_REVERSE_RIGHT3 = true; 
  public static final boolean DT_3_MOTORS_PER_SIDE = true;

	Joystick _joy = new Joystick(0);
	
  /** Used to create string thoughout loop */
	StringBuilder _sb = new StringBuilder();
	int _loops = 0;
	
  /** Track button state for single press event */
  boolean _lastButton1 = false;
  boolean _last_right_bumper = false;
  boolean _last_left_bumper = false;

	/** Save the target position to servo to */
	double targetPositionRotations;


  final double TICKS_PER_REV = 8192.0;
  final double DEGREES_PER_REV = 360.0;
  final double NUM_REVOLUTIONS = 1.0; 
  
	/**
	  *Gains used in Positon Closed Loop, to be adjusted accordingly
    * Gains(kp, ki, kd, kf, izone, peak output);

                                  kP    kI    kD   kF IZone PeakOut*/
  final Gains kGains = new Gains(2.65, 0.000, 0.06, 0.0, 0, 0.6 ); //kD=0.06, kI=0.00001

  /**
   * Converts a setpoint in degrees to IMU 'encoder ticks'
   * @param setpoint
   * @return
   */
  private double degrees_to_ticks(double setpoint) {
    return (setpoint / DEGREES_PER_REV) * TICKS_PER_REV;
  }

  private double ticks_to_degrees(double setpoint) {
    return (setpoint / TICKS_PER_REV) * DEGREES_PER_REV;
  }

  private double degrees_per_sec_to_ticks_per_100ms(double setpoint) {
    return (degrees_to_ticks(setpoint) / 10.0);
  }

  private double ticks_per_100ms_to_degrees_per_sec(double setpoint) {
    return (ticks_to_degrees(setpoint) * 10.0);
  }

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    /* Factory Default all hardware to prevent unexpected behaviour */
    _leftMotor1.configFactoryDefault();
    _leftMotor2.configFactoryDefault();
    _leftMotor3.configFactoryDefault();
    _rightMotor1.configFactoryDefault();
    _rightMotor2.configFactoryDefault();
    _rightMotor3.configFactoryDefault();
    
    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
    CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    _leftMotor1.configSupplyCurrentLimit(talonCurrentLimit);
    _leftMotor2.configSupplyCurrentLimit(talonCurrentLimit);
    _leftMotor3.configSupplyCurrentLimit(talonCurrentLimit);
    _rightMotor1.configSupplyCurrentLimit(talonCurrentLimit);
    _rightMotor2.configSupplyCurrentLimit(talonCurrentLimit);
    _rightMotor3.configSupplyCurrentLimit(talonCurrentLimit);

    _leftMotor1.setNeutralMode(NeutralMode.Brake);
    _leftMotor1.setNeutralMode(NeutralMode.Coast);
    _leftMotor1.setNeutralMode(NeutralMode.Coast);
    _rightMotor1.setNeutralMode(NeutralMode.Brake);
    _rightMotor1.setNeutralMode(NeutralMode.Coast);
    _rightMotor1.setNeutralMode(NeutralMode.Coast);

    /* Configure the Pigeon IMU to the other remote slot available on the right Talon */
    _rightMotor1.configRemoteFeedbackFilter(_pidgey.getDeviceID(),
                                        RemoteSensorSource.Pigeon_Yaw,
                                        0,  //remote sensor position
                                        Constants.kTimeoutMs);

		/* Config the sensor used for Primary PID and sensor direction */
    _rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 
                                        Constants.kPIDLoopIdx,
                                        Constants.kTimeoutMs);

		/* Ensure sensor is positive when output is positive */
		_rightMotor1.setSensorPhase(Constants.kSensorPhase);

		/**
		 * Set based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. 
		 */ 
    _rightMotor1.setInverted(Constants.kMotorInvert);
    _rightMotor2.setInverted(Constants.kMotorInvert);
    _rightMotor3.setInverted(Constants.kMotorInvert);
    _leftMotor1.setInverted(Constants.kMotorInvert);
    _leftMotor2.setInverted(Constants.kMotorInvert);
    _leftMotor3.setInverted(Constants.kMotorInvert);

		/* Config the peak and nominal outputs, 12V means full */
		_rightMotor1.configNominalOutputForward(0.05, Constants.kTimeoutMs);
		_rightMotor1.configNominalOutputReverse(0.05, Constants.kTimeoutMs);
		_rightMotor1.configPeakOutputForward(kGains.kPeakOutput, Constants.kTimeoutMs);
		_rightMotor1.configPeakOutputReverse(-kGains.kPeakOutput, Constants.kTimeoutMs);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		_rightMotor1.configAllowableClosedloopError((int)(degrees_to_ticks(2.0)), Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		_rightMotor1.config_kF(Constants.kPIDLoopIdx, kGains.kF, Constants.kTimeoutMs);
		_rightMotor1.config_kP(Constants.kPIDLoopIdx, kGains.kP, Constants.kTimeoutMs);
		_rightMotor1.config_kI(Constants.kPIDLoopIdx, kGains.kI, Constants.kTimeoutMs);
    _rightMotor1.config_kD(Constants.kPIDLoopIdx, kGains.kD, Constants.kTimeoutMs);
    
    _rightMotor1.configMotionCruiseVelocity((int) (degrees_per_sec_to_ticks_per_100ms(200)), Constants.kTimeoutMs);
    _rightMotor1.configMotionAcceleration((int) (degrees_per_sec_to_ticks_per_100ms(240)), Constants.kTimeoutMs);
		// /**
		//  * Grab the 360 degree position of the MagEncoder's absolute
		//  * position, and intitally set the relative sensor to match.
		//  */
		// int absolutePosition = _rightMotor1.getSensorCollection().getPulseWidthPosition();

		// /* Mask out overflows, keep bottom 12 bits */
		// absolutePosition &= 0xFFF;
		// if (Constants.kSensorPhase) { absolutePosition *= -1; }
		// if (Constants.kMotorInvert) { absolutePosition *= -1; }
		
		// /* Set the quadrature (relative) sensor to match absolute */
    // _rightMotor1.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
 
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    commonLoop();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  void commonLoop() {
		/* Gamepad processing */
		double leftYstick = _joy.getY();
		boolean button1 = _joy.getRawButton(1);	// A-Button
    boolean button2 = _joy.getRawButton(2);	// B-Button
    boolean button3 = _joy.getRawButton(3); //X button
    boolean left_bumper = _joy.getRawButton(5); //left bumper
    boolean right_bumper = _joy.getRawButton(6); //right bumper

    /* Set motor followers */
    _rightMotor2.follow(_rightMotor1, FollowerType.PercentOutput);
    _rightMotor3.follow(_rightMotor1, FollowerType.PercentOutput);
    _leftMotor1.follow(_rightMotor1, FollowerType.PercentOutput);
    _leftMotor2.follow(_rightMotor1, FollowerType.PercentOutput);
    _leftMotor3.follow(_rightMotor1, FollowerType.PercentOutput);

		/* Get Talon/Victor's current output percentage */
		double motorOutput = _rightMotor1.getMotorOutputPercent();

    if (button3) {
      _rightMotor1.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
      _leftMotor1.getSensorCollection().setIntegratedSensorPosition (0, Constants.kTimeoutMs);
      _pidgey.setYaw(0, Constants.kTimeoutMs);
      _pidgey.setAccumZAngle(0, Constants.kTimeoutMs);
      // System.out.println("[Quadrature Encoders + Pigeon] All sensors are zeroed.\n");
    }

		/* Deadband gamepad */
		if (Math.abs(leftYstick) < 0.10) {
			/* Within 10% of zero */
			leftYstick = 0;
		}

		/* Prepare line to print */
		_sb.append("\tout:");
		/* Cast to int to remove decimal places */
		_sb.append((int) (motorOutput * 100));
		_sb.append("%");	// Percent

		_sb.append("\tpos:");
		_sb.append(ticks_to_degrees(_rightMotor1.getSelectedSensorPosition(0)));
    _sb.append("u"); 	// Native units
    
    _sb.append("\tvel:");
    _sb.append(ticks_per_100ms_to_degrees_per_sec(_rightMotor1.getSelectedSensorVelocity(0)
    ));
		_sb.append("u"); 	// Native units

		/**
		 * When button 1 is pressed, perform Position Closed Loop to selected position,
		 * indicated by Joystick position x10, [-10, 10] rotations
		 */
		if (!_lastButton1 && button1) {
			/* Position Closed Loop */

			/* number of revs * ticks/rev in either direction */
			targetPositionRotations = leftYstick * NUM_REVOLUTIONS * TICKS_PER_REV;
			_rightMotor1.set(ControlMode.MotionMagic, targetPositionRotations);
		} else if (!_last_left_bumper && left_bumper) {
			/* Drive -90.0 degrees */

			targetPositionRotations = degrees_to_ticks(-90.0);
			_rightMotor1.set(ControlMode.MotionMagic, targetPositionRotations, DemandType.ArbitraryFeedForward, 0.0);
		} else if (!_last_right_bumper && right_bumper) {
			/* Drive -90.0 degrees */

			targetPositionRotations = degrees_to_ticks(90.0);
			_rightMotor1.set(ControlMode.MotionMagic, targetPositionRotations, DemandType.ArbitraryFeedForward, 0.0);
		}



		/* When button 2 is held, just straight drive */
		if (button2) {
			/* Percent Output */

			_rightMotor1.set(ControlMode.PercentOutput, leftYstick);
		}

		/* If Talon is in position closed-loop, print some more info */
		if (_rightMotor1.getControlMode() == ControlMode.MotionMagic) {
			/* ppend more signals to print when in speed mode. */
      _sb.append("\terr:");
      _sb.append(ticks_to_degrees(targetPositionRotations-_rightMotor1.getSelectedSensorPosition(0)));
			// _sb.append(ticks_to_degrees(_rightMotor1.getClosedLoopError(0)));
			_sb.append("u");	// Native Units

			_sb.append("\ttrg:");
			_sb.append(ticks_to_degrees(targetPositionRotations));
			_sb.append("u");	/// Native Units
		}

		/**
		 * Print every ten loops, printing too much too fast is generally bad
		 * for performance.
		 */
		if (++_loops >= 10) {
			_loops = 0;
			System.out.println(_sb.toString());
		}

		/* Reset built string for next loop */
		_sb.setLength(0);
		
		/* Save button state for on press detect */
    _lastButton1 = button1;
    _last_left_bumper = left_bumper;
    _last_right_bumper = right_bumper;
    }
}
