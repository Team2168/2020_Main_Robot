package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.team2168.RobotMap;
import org.team2168.commands.drivetrain.DriveWithJoystick;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.command.Subsystem;


public class Drivetrain extends Subsystem {  
  private static TalonFX _leftMotor1;
  private static TalonFX _leftMotor2;
  private static TalonFX _leftMotor3;
  private static TalonFX _rightMotor1;
  private static TalonFX _rightMotor2;
  private static TalonFX _rightMotor3;

  private static Drivetrain instance = null;

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

  /**
   * Default constructors for Drivetrain
   */
  private Drivetrain() {
      System.out.println("CAN Comp Bot Drivetrain enabled - 6 motors");
      _leftMotor1 = new TalonFX(RobotMap.DRIVETRAIN_LEFT_MOTOR_1_PDP);
      _leftMotor2 = new TalonFX(RobotMap.DRIVETRAIN_LEFT_MOTOR_2_PDP);
      _leftMotor3 = new TalonFX(RobotMap.DRIVETRAIN_LEFT_MOTOR_3_PDP);
      _rightMotor1 = new TalonFX(RobotMap.DRIVETRAIN_RIGHT_MOTOR_1_PDP);
      _rightMotor2 = new TalonFX(RobotMap.DRIVETRAIN_RIGHT_MOTOR_2_PDP);
      _rightMotor3 = new TalonFX(RobotMap.DRIVETRAIN_RIGHT_MOTOR_3_PDP);
    
      talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
      CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

      _leftMotor1.configSupplyCurrentLimit(talonCurrentLimit);
      _leftMotor2.configSupplyCurrentLimit(talonCurrentLimit);
      _leftMotor3.configSupplyCurrentLimit(talonCurrentLimit);
      _rightMotor1.configSupplyCurrentLimit(talonCurrentLimit);
      _rightMotor2.configSupplyCurrentLimit(talonCurrentLimit);
      _rightMotor3.configSupplyCurrentLimit(talonCurrentLimit);

      _leftMotor1.setNeutralMode(NeutralMode.Brake);
      _leftMotor2.setNeutralMode(NeutralMode.Coast);
      _leftMotor3.setNeutralMode(NeutralMode.Coast);
      _rightMotor1.setNeutralMode(NeutralMode.Brake);
      _rightMotor2.setNeutralMode(NeutralMode.Coast);
      _rightMotor3.setNeutralMode(NeutralMode.Coast);

     //Log sensor data
     //ConsolePrinter.putNumber("DTRight1MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR_1_PDP);}, true, false);
     //ConsolePrinter.putNumber("DTRight2MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR_2_PDP);}, true, false);
     //ConsolePrinter.putNumber("DTRight3MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR_3_PDP);}, true, false);
     //ConsolePrinter.putNumber("DTLeft1MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR_1_PDP);}, true, false);
     //ConsolePrinter.putNumber("DTLeft2MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR_2_PDP);}, true, false);
     //ConsolePrinter.putNumber("DTLeft3MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR_3_PDP);}, true, false);
    
  }

  /**
   * @returns An instance of the Drivetrain subsystem
   */
  public static Drivetrain getInstance() {
    if (instance == null)
      instance = new Drivetrain();

    return instance;
  }

  /**
   * sets the speed of left motor 1
   * 
   * @param speed between -1 and 1 negative is reverse, positive is
   *        forward, 0 is stationary
   */
  private void driveleftMotor1(double speed) {
    if (DT_REVERSE_LEFT1)
      speed = -speed;

    _leftMotor1.set(ControlMode.PercentOutput, speed);

  }

  /**
   * sets the speed of left motor 2
   * 
   * @param speed between -1 and 1 negative is reverse, positive is
   *        forward, 0 is stationary
   */
  private void driveleftMotor2(double speed) {
    if (DT_REVERSE_LEFT2)
      speed = -speed;

    _leftMotor2.set(ControlMode.PercentOutput, speed);
  }

  /**
   * sets the speed of left motor 3
   * 
   * @param speed between -1 and 1 negative is reverse, positive is
   *        forward, 0 is stationary
   */
  private void driveleftMotor3(double speed) {
    if (DT_REVERSE_LEFT3)
      speed = -speed;

    _leftMotor3.set(ControlMode.PercentOutput, speed);
  }

  /**
   * sets the speed of left motors 1, 2, and 3
   * 
   * @param speed 1.0 to -1.0. negative is reverse, positive is
   *              forward, 0 is stationary
   */
  public void driveLeft(double speed) {
    if (DT_3_MOTORS_PER_SIDE) {
      driveleftMotor1(speed);
      driveleftMotor2(speed);
      driveleftMotor3(speed);
    } else {
      driveleftMotor1(speed);
      driveleftMotor2(speed);
    }
  }
/**
   * sets the speed of right motor 1
   * 
   * @param speed between -1 and 1 negative is reverse, positive is
   *        forward, 0 is stationary
   */
  private void driverightMotor1(double speed) {
    if (DT_REVERSE_RIGHT1)
      speed = -speed;

    _rightMotor1.set(ControlMode.PercentOutput, speed);
  }

  /**
   * sets the speed of right motor 2
   * 
   * @param speed between -1 and 1 negative is reverse, positive is
   *        forward, 0 is stationary
   */
  private void driverightMotor2(double speed) {
    if (DT_REVERSE_RIGHT2)
      speed = -speed;

    _rightMotor2.set(ControlMode.PercentOutput, speed);
  }

  /**
   * sets the speed of right motor 3
   * 
   * @param speed between -1 and 1 negative is reverse, positive is
   *        forward, 0 is stationary
   */
  private void driverightMotor3(double speed) {
    if (DT_REVERSE_RIGHT3)
      speed = -speed;

    _rightMotor3.set(ControlMode.PercentOutput, speed);
  }
  
  public void driveRight(double speed) {
    if(DT_3_MOTORS_PER_SIDE)
    {
      driverightMotor1(speed);
      driverightMotor2(speed);
      driverightMotor3(speed);
    } else {
      driverightMotor1(speed);
      driverightMotor2(speed);
    }
  }

  public void tankDrive(double leftSpeed, double rightSpeed)
  {
    driveLeft(leftSpeed);
    driveRight(rightSpeed);
  }


  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoystick());
  }

}