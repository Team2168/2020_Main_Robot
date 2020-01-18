/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team2168.Gains;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class ExampleVelocityClosedLoopSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  
  TalonSRX _talon = new TalonSRX(11);
  TalonSRX _talonFollow = new TalonSRX(10);
  private static ExampleVelocityClosedLoopSubsystem _instance; 

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
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
    public static final int kTimeoutMs = 30;
	
	public static final double max_velocity = 8000.0;
/**
   * Convert target RPM to ticks / 100ms.
   * 256*4x (quadrature encoder) Ticks/Rev *  RPM / 600 100ms/min in either direction:
   * velocity setpoint is in units/100ms
   */
  final double TICKS_PER_REV = 256.0 * 4.0; //one event per edge on each quadrature channel
  final double TICKS_PER_100MS = TICKS_PER_REV / 600.0;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 7200 represents Velocity units at 100% output
     * 
	 * 	                                      kP    kI   kD          kF               Iz   PeakOut */
  final Gains kGains_Velocity = new Gains( 0.775, 0.000, 0, 0.17825/TICKS_PER_100MS,  300,  1.00); // kF = 1023*0.00016/ticks_per_100ms

  private ExampleVelocityClosedLoopSubsystem()
  {
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
    
        /* Config the peak and nominal outputs */
        _talon.configNominalOutputForward(0, kTimeoutMs);
        _talon.configNominalOutputReverse(0, kTimeoutMs);
        _talon.configPeakOutputForward(1, kTimeoutMs);
        _talon.configPeakOutputReverse(-1, kTimeoutMs);
    
        _talon.enableCurrentLimit(true);
        _talonFollow.enableCurrentLimit(true);
        _talon.configContinuousCurrentLimit(30);
        _talonFollow.configContinuousCurrentLimit(30);
        _talon.configPeakCurrentDuration(500);
        _talonFollow.configPeakCurrentDuration(500);
        _talon.configPeakCurrentLimit(60);
        _talon.configPeakCurrentLimit(60);
    
        /* Config the Velocity closed loop gains in slot0 */
        _talon.config_kF(kPIDLoopIdx, kGains_Velocity.kF, kTimeoutMs);
        _talon.config_kP(kPIDLoopIdx, kGains_Velocity.kP, kTimeoutMs);
        _talon.config_kI(kPIDLoopIdx, kGains_Velocity.kI, kTimeoutMs);
        _talon.config_kD(kPIDLoopIdx, kGains_Velocity.kD, kTimeoutMs);
    
        //controls acceleration 
        //_talon.configClosedloopRamp(0.1);
        //_talon.configOpenloopRamp(1.0);

        //set second motor as a follower
        _talonFollow.follow(_talon, FollowerType.PercentOutput);

        ConsolePrinter.putNumber("Velocity", () -> {return getVelocity();}, true, false);
        ConsolePrinter.putNumber("Error", () -> {return getError();}, true, false);
        ConsolePrinter.putNumber("Motor Output Percent", () -> {return _talon.getMotorOutputPercent();}, true, false);
        ConsolePrinter.putNumber("Setpoint", () -> {return _talon.getClosedLoopTarget()/TICKS_PER_100MS;}, true, false);
    

  }

    /**
   * singleton constructor of the ExampleVelocityClosedLoopSubsystem 
   * @return
   */
  public static ExampleVelocityClosedLoopSubsystem getInstance()
  {
    if (_instance == null)
      _instance = new ExampleVelocityClosedLoopSubsystem();
    return _instance;
  }
  
  public void drive(double speed)
  {
    _talon.set(ControlMode.PercentOutput, speed);
  }


  public double getVelocity()
  {
    return _talon.getSelectedSensorVelocity(kPIDLoopIdx)/TICKS_PER_100MS;
  }

  public void setSetPoint(double setPoint)
  {
    _talon.set(ControlMode.Velocity, setPoint*TICKS_PER_100MS);
  }

  public double getError()
  {
    return _talon.getClosedLoopError(kPIDLoopIdx)/TICKS_PER_100MS;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
