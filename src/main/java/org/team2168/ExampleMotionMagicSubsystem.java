/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class ExampleMotionMagicSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 30;

	/**
	 * Gains used in Motion Magic, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    static final Gains kGains = new Gains(0.57, 0.0, 0.0, 0.2, 0, 1.0);
  /**
   * Convert target RPM to ticks / 100ms.
   * 256*4x (quadrature encoder) Ticks/Rev *  RPM / 600 100ms/min in either direction:
   * velocity setpoint is in units/100ms
   */
  final double TICKS_PER_REV = 256.0 * 4.0; //one event per edge on each quadrature channel
  final double TICKS_PER_100MS = TICKS_PER_REV / 600.0;
  final double NUM_REVOLUTIONS = 32.0;
  public TalonSRX _talon;
  public TalonSRX _talonFollow;

  private static ExampleMotionMagicSubsystem _instance;


  private ExampleMotionMagicSubsystem()
  {
    _talon = new TalonSRX(11);
    _talonFollow = new TalonSRX(10);
    
    /* Factory Default all hardware to prevent unexpected behaviour */
    _talon.configFactoryDefault();

    /* Config sensor used for Primary PID [Velocity] */
        _talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
                                            kPIDLoopIdx, 
                                            kTimeoutMs);

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
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

		/* Set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, kTimeoutMs);
		_talon.configNominalOutputReverse(0, kTimeoutMs);
		_talon.configPeakOutputForward(1, kTimeoutMs);
		_talon.configPeakOutputReverse(-1, kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		_talon.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		_talon.config_kF(kSlotIdx, kGains.kF, kTimeoutMs);
		_talon.config_kP(kSlotIdx, kGains.kP, kTimeoutMs);
		_talon.config_kI(kSlotIdx, kGains.kI, kTimeoutMs);
		_talon.config_kD(kSlotIdx, kGains.kD, kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity((int) (1600.0*TICKS_PER_100MS), kTimeoutMs);
		_talon.configMotionAcceleration((int) (800.0*TICKS_PER_100MS), kTimeoutMs);

		/* Zero the sensor */
    _talon.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);

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
   * singleton constructor of the ExampleMotionMagicSubsystem 
   * @return
   */
  public static ExampleMotionMagicSubsystem getInstance()
  {
    if (_instance == null)
      _instance = new ExampleMotionMagicSubsystem();
    return _instance;
  }

  public void setTalonFollower()
  {
    _talonFollow.follow(Robot.exampleMotionMagicSubsystem._talon, FollowerType.PercentOutput);

  }

  public double getMotorOutputPercent()
  {
    return _talon.getMotorOutputPercent();
  }

  public double getPosition()
  {
    return _talon.getSelectedSensorPosition(kPIDLoopIdx)/TICKS_PER_REV;
  }

  public double getVelocity()
  {
    return _talon.getSelectedSensorVelocity(kPIDLoopIdx)/TICKS_PER_100MS;
  }

  public void setSetPoint(double setPoint)
  {
    _talon.set(ControlMode.MotionMagic, setPoint*TICKS_PER_REV);
  }

  public double getError()
  {
    return _talon.getClosedLoopError(kPIDLoopIdx)/TICKS_PER_REV;
  }

  public boolean reachedSetpoint(double errorTolerance)
  {
    return Math.abs(getError()) < errorTolerance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
