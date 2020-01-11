/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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

  TalonSRX _talon = new TalonSRX(11);
  TalonSRX _talonFollow = new TalonSRX(10);
  Joystick _joy = new Joystick(0);

  StringBuilder _sb = new StringBuilder();

  int _loops = 0;
/**
   * Convert target RPM to ticks / 100ms.
   * 256*4x (quadrature encoder) Ticks/Rev *  RPM / 600 100ms/min in either direction:
   * velocity setpoint is in units/100ms
   */
  double ticks_per_rev = 256.0 * 4.0; //one event per edge on each quadrature channel
  double ticks_per_100ms = ticks_per_rev / 600.0;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 7200 represents Velocity units at 100% output
     * 
	 * 	                                      kP    kI   kD          kF               Iz   PeakOut */
  final Gains kGains_Velocity = new Gains( 0.775, 0.000, 0, 0.17825/ticks_per_100ms,  300,  1.00); // kF = 1023*0.00016/ticks_per_100ms


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
    _talon.configFactoryDefault();

    /* Config sensor used for Primary PID [Velocity] */
        _talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
                                            Constants.kPIDLoopIdx, 
                                            Constants.kTimeoutMs);

        /**
     * Phase sensor accordingly. 
         * Positive Sensor Reading should match Green (blinking) Leds on Talon
         */
    _talon.setSensorPhase(true);

    /***
     * invert motors if necessary
     */
    _talon.setInverted(true);
    _talonFollow.setInverted(true);

    /* Config the peak and nominal outputs */
    _talon.configNominalOutputForward(0, Constants.kTimeoutMs);
    _talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
    _talon.configPeakOutputForward(1, Constants.kTimeoutMs);
    _talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    _talon.enableCurrentLimit(true);
    _talonFollow.enableCurrentLimit(true);
    _talon.configContinuousCurrentLimit(30);
    _talonFollow.configContinuousCurrentLimit(30);
    _talon.configPeakCurrentDuration(500);
    _talonFollow.configPeakCurrentDuration(500);
    _talon.configPeakCurrentLimit(60);
    _talon.configPeakCurrentLimit(60);

    /* Config the Velocity closed loop gains in slot0 */
    _talon.config_kF(Constants.kPIDLoopIdx, kGains_Velocity.kF, Constants.kTimeoutMs);
    _talon.config_kP(Constants.kPIDLoopIdx, kGains_Velocity.kP, Constants.kTimeoutMs);
    _talon.config_kI(Constants.kPIDLoopIdx, kGains_Velocity.kI, Constants.kTimeoutMs);
    _talon.config_kD(Constants.kPIDLoopIdx, kGains_Velocity.kD, Constants.kTimeoutMs);

    //_talon.configClosedloopRamp(0.1);
    _talon.configOpenloopRamp(1.0);
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
    /* Get gamepad axis */
    double leftYstick = -1 * _joy.getY();
    _talonFollow.follow(_talon, FollowerType.PercentOutput);

    double speed_limit = 6000.0; //RPM;

		/* Get Talon/Victor's current output percentage */
		double motorOutput = _talon.getMotorOutputPercent();
		
		/* Prepare line to print */
		_sb.append("\tout:");
		/* Cast to int to remove decimal places */
		_sb.append((int) (motorOutput * 100));
		_sb.append("%");	// Percent

		_sb.append("\tspd:");
		_sb.append(_talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx)/ticks_per_100ms);
		_sb.append("u"); 	// Native units

    /** 
		 * When button 1 is held, start and run Velocity Closed loop.
		 * Velocity Closed Loop is controlled by joystick position
		 */
		if (_joy.getRawButton(1)) {
			/* Velocity Closed Loop */
			double targetVelocity_UnitsPer100ms = leftYstick * speed_limit * ticks_per_100ms;
			_talon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);

			/* Append more signals to print when in speed mode. */
			_sb.append("\terr:");
			_sb.append(_talon.getClosedLoopError(Constants.kPIDLoopIdx)/ticks_per_100ms);
			_sb.append("\ttrg:");
      _sb.append(targetVelocity_UnitsPer100ms/ticks_per_100ms);
      
      SmartDashboard.putNumber("Out%", (int) (motorOutput * 100));
      SmartDashboard.putNumber("speed", _talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx)/ticks_per_100ms);
      SmartDashboard.putNumber("error", _talon.getClosedLoopError(Constants.kPIDLoopIdx)/ticks_per_100ms);
      SmartDashboard.putNumber("target", targetVelocity_UnitsPer100ms/ticks_per_100ms);
		} else {
			/* Percent Output */

      _talon.set(ControlMode.PercentOutput, leftYstick);
      //_talonFollow.set(ControlMode.PercentOutput, leftYstick);
		}

    /* Print built string every 10 loops */
		if (++_loops >= 10) {
			_loops = 0;
			System.out.println(_sb.toString());
        }
        /* Reset built string */
		_sb.setLength(0);

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
