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
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team2168.Gains;
import org.team2168.RobotMap;
import org.team2168.commands.climber_comm.DriveClimberWithJoystick;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem {

  public static Climber instance = null;

  public static double holdingSpeed; 
  private TalonSRX climberMotor1;
  private TalonSRX climberMotor2;
  public Solenoid climberSolenoid;

  private final boolean CLIMBER_MOTOR_1_REVERSE = false;
  private final boolean CLIMBER_MOTOR_2_REVERSE = false;
  public static final boolean CLIMBER_ENABLE_HIGHT_HOLD = true;

  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 40; //amps
  private final double TRIGGER_THRESHOLD_LIMIT = 60; //amp
  private final double TRIGGER_THRESHOLD_TIME = 200; //ms

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
  final double GEAR_RATIO = 1.0; //TODO SET


  private Climber() {
    climberMotor1 = new TalonSRX(RobotMap.CLIMBER_MOTOR_1);
    climberMotor2 = new TalonSRX(RobotMap.CLIMBER_MOTOR_2);
    climberSolenoid = new Solenoid(RobotMap.CLIMBER_RATCHET);
    /* Factory Default all hardware to prevent unexpected behaviour */
    climberMotor1.configFactoryDefault();
    /* Configure the left Talon's selected sensor as local QuadEncoder */
    climberMotor1.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,				// Local Feedback Source
                          kPIDLoopIdx,					// PID Slot for Source [0, 1]
                          kTimeoutMs);					// Configuration Timeout

    /**
    * Phase sensor accordingly. 
    * Positive Sensor Reading should match Green (blinking) Leds on Talon
    */
    climberMotor1.setSensorPhase(true);

    /***
     * invert motors if necessary
     */
    climberMotor1.setInverted(CLIMBER_MOTOR_1_REVERSE);
    climberMotor2.setInverted(CLIMBER_MOTOR_2_REVERSE);

        /* Set relevant frame periods to be at least as fast as periodic rate */
    climberMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
    climberMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

    /* Set the peak and nominal outputs */
    climberMotor1.configNominalOutputForward(0, kTimeoutMs);
    climberMotor1.configNominalOutputReverse(0, kTimeoutMs);
    climberMotor1.configPeakOutputForward(1, kTimeoutMs);
    climberMotor1.configPeakOutputReverse(-1, kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    climberMotor1.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
    climberMotor1.config_kF(kSlotIdx, kGains.kF, kTimeoutMs);
    climberMotor1.config_kP(kSlotIdx, kGains.kP, kTimeoutMs);
    climberMotor1.config_kI(kSlotIdx, kGains.kI, kTimeoutMs);
    climberMotor1.config_kD(kSlotIdx, kGains.kD, kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    climberMotor1.configMotionCruiseVelocity((int) (1600.0*TICKS_PER_100MS), kTimeoutMs); //todo set
    climberMotor1.configMotionAcceleration((int) (800.0*TICKS_PER_100MS), kTimeoutMs); //todo set

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
    CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    climberMotor1.configSupplyCurrentLimit(talonCurrentLimit);
    climberMotor2.configSupplyCurrentLimit(talonCurrentLimit);

    /* Zero the sensor */
    climberMotor1.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
                    

  }
  
  /**
   * This method creates a new instance of the climber, allowing other 
   * programs, i.e. the commands, to utilize it.
   * @return - Returns the new instance of the climber.
   */
  public static Climber getInstance()
  {
    if (instance == null)
      instance = new Climber();
    return instance;
  }
  
  /** 
   *  This method will set the climbers motors to a new speed, allowing
   * the motors to be operated. 
   * @param speed - 1 is to raise the climber, and -1 is to lower it.
   */
  
  public void driveClimberMotors(double speed){
    driveClimberMotor1(speed);
    driveClimberMotor2(speed);
  }

  /**
   * This method is mostly used for testing only motor one.
   * @param speed - 1 is to raise the climber, and -1 is to lower it.
   */
  public void driveClimberMotor1(double speed){
    // if(CLIMBER_MOTOR_1_REVERSE){
    //   speed = -speed;
    // }
      
    climberMotor1.set(ControlMode.PercentOutput, speed);
  }
  /**
   * This method is used for testing only motor two.
   * @param speed - 1 is to raise the climber, and -1 is to lower it.
   */
  public void driveClimberMotor2(double speed){
    // if(CLIMBER_MOTOR_2_REVERSE){
    //   speed = -speed;
    // }
    climberMotor2.set(ControlMode.PercentOutput, speed);
  }

  /** 
   * This method allows the ratchet to extend, preventing the climber
   * from moving from the lowered position.
   */
  public void extendRatchet(){
    climberSolenoid.set(false);
  }

  /** 
   * This method allows the rathcet to retract, allowing the climber to move
   *  to a raised position.
   */
  public void retractRatchet(){
    climberSolenoid.set(true);
  }

    /**
   * returns whether or not the ratchet is extended
   * @return
   */
  public boolean isRatchetExtended(){
    return !climberSolenoid.get();
  }

  /**
   * This will allow the program to return whether the ratchet is extended or not.
   * @return
   */
  public boolean isRatchetRetracted(){
    return climberSolenoid.get();
  
  }

  public double getPosition()
  {
    return climberMotor1.getSelectedSensorPosition(kPIDLoopIdx)/(TICKS_PER_REV*GEAR_RATIO);
  }

  public double getVelocity()
  {
    return climberMotor1.getSelectedSensorVelocity(kPIDLoopIdx)/(TICKS_PER_100MS*GEAR_RATIO);
  }

  public void setSetPoint(double setPoint)
  {
    climberMotor1.set(ControlMode.MotionMagic, setPoint*TICKS_PER_REV*GEAR_RATIO);
    climberMotor2.follow(climberMotor1, FollowerType.PercentOutput);
  }

  public double getErrorPosition()
  {
    return (climberMotor1.getActiveTrajectoryPosition()-climberMotor1.getSelectedSensorPosition(kPIDLoopIdx))/(TICKS_PER_REV*GEAR_RATIO);
    //return climberMotor1.getClosedLoopError(kPIDLoopIdx)/TICKS_PER_REV;--only for nonMotionMagic or nonMotion Profile
  }

  public void zeroEncoder()
  {
    climberMotor1.setSelectedSensorPosition(0);
  }

  @Override
  /**
   * This sets the default command to drive via a joystick.
   */
  public void initDefaultCommand() {
    setDefaultCommand(new DriveClimberWithJoystick());
  }

}
