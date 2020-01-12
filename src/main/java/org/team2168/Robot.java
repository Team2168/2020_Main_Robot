/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
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

  /** Track button state for single press event */
  boolean _lastButton1 = false;
  
  /**target position */
  double targetPos;
/**
   * Convert target RPM to ticks / 100ms.
   * 256*4x (quadrature encoder) Ticks/Rev *  RPM / 600 100ms/min in either direction:
   * velocity setpoint is in units/100ms
   */
  final double TICKS_PER_REV = 256.0 * 4.0; //one event per edge on each quadrature channel
  final double TICKS_PER_100MS = TICKS_PER_REV / 600.0;
  final double NUM_REVOLUTIONS = 10.0;

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

    		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		_talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		_talon.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_talon.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_talon.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_talon.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity((int) (1600.0*TICKS_PER_100MS), Constants.kTimeoutMs);
		_talon.configMotionAcceleration((int) (1600.0*TICKS_PER_100MS), Constants.kTimeoutMs);

		/* Zero the sensor */
    _talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    _talon.enableCurrentLimit(true);
    _talonFollow.enableCurrentLimit(true);
    _talon.configContinuousCurrentLimit(30);
    _talonFollow.configContinuousCurrentLimit(30);
    _talon.configPeakCurrentDuration(500);
    _talonFollow.configPeakCurrentDuration(500);
    _talon.configPeakCurrentLimit(60);
    _talon.configPeakCurrentLimit(60);
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

    	/* Get gamepad axis - forward stick is positive */
    double leftYstick = -1.0 * _joy.getY();
    boolean button1 = _joy.getRawButton(1);	// A-Button
    boolean button2 = _joy.getRawButton(2);	// B-Button

    _talonFollow.follow(_talon, FollowerType.PercentOutput);

    /* Get current Talon SRX motor output */
    double motorOutput = _talon.getMotorOutputPercent();

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
    _sb.append("\tVel:");
    _sb.append(_talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx)/TICKS_PER_100MS);
    _sb.append("\tpos:");
    _sb.append(_talon.getSelectedSensorPosition(Constants.kPIDLoopIdx)/TICKS_PER_REV);
    _sb.append("u"); 	// Native units

    /**
     * Peform Motion Magic when Button 1 is held,
     * else run Percent Output, which can be used to confirm hardware setup.
     */
    if (button1) {
      /* Motion Magic */ 
      
      /*4096 ticks/rev * 10 Rotations in either direction */
      targetPos = leftYstick * TICKS_PER_REV * NUM_REVOLUTIONS;
      _talon.set(ControlMode.MotionMagic, targetPos);


    } 
    else if (button2)
    {
      /* Percent Output */

      _talon.set(ControlMode.PercentOutput, leftYstick);
    }

    if (_talon.getControlMode() == ControlMode.MotionMagic)
    {
          /* Append more signals to print when in speed mode */
          _sb.append("\terr:");
          _sb.append(_talon.getClosedLoopError(Constants.kPIDLoopIdx)/TICKS_PER_REV);
          _sb.append("\ttrg:");
          _sb.append(targetPos/TICKS_PER_REV);
    }
    /* Instrumentation */
  //  Instrum.Process(_talon, _sb); 
   
    /* Periodically print to console */
    if (++_loops >= 10) {
      _loops = 0;
      System.out.println(_sb.toString());
    }

    /* Reset created string for next loop */
    _sb.setLength(0);

    /* 10 Ms timeout, allow CAN Frames to process */
    try { TimeUnit.MILLISECONDS.sleep(10); } 	
    catch (Exception e) { /* Do Nothing */ }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
