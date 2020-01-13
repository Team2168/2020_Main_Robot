package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

//mport org.team2168.PID.sensors.NavX;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.controllers.PIDPosition;
import org.team2168.PID.controllers.PIDSpeed;
import org.team2168.PID.sensors.AverageEncoder;
import org.team2168.PID.sensors.IMU;
import org.team2168.PID.sensors.Limelight;
import org.team2168.utils.TCPSocketSender;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;


public class Drivetrain extends Subsystem
{  
  private static TalonFX _leftMotor1;
  private static TalonFX _leftMotor2;
  private static TalonFX _leftMotor3;
  private static TalonFX _rightMotor1;
  private static TalonFX _rightMotor2;
  private static TalonFX _rightMotor3;

  private AverageEncoder _drivetrainLeftEncoder;
  private AverageEncoder _drivetrainRightEncoder;

  //Leave these wihtout _name convention to work with past code base
  private double RightMotor1FPS;
  private double RightMotor2FPS;
  private double RightMotor3FPS;
  private double leftMotor1FPS;
  private double lefttMotor1FPS;
  private double leftMotor3FPS;
  public IMU imu;



  // declare position/speed controllers
  public PIDPosition driveTrainPosController;
  public PIDPosition rotateController;
  public PIDPosition rotateDriveStraightController;

  public PIDPosition rightPosController;
  public PIDPosition leftPosController;

  // declare speed controllers
  public PIDSpeed rightSpeedController;
  public PIDSpeed leftSpeedController;

  public Limelight limelight;
  public PIDPosition limelightPosController;

  private static Drivetrain instance = null;

  // declare TCP severs...ONLY FOR DEBUGGING PURPOSES, SHOULD BE REMOVED FOR
  // COMPITITION
  TCPSocketSender TCPdrivePosController;
  TCPSocketSender TCPrightSpeedController;
  TCPSocketSender TCPleftSpeedController;
  TCPSocketSender TCProtateController;
  TCPSocketSender TCPleftPosController;
  TCPSocketSender TCPrightPosController;
  TCPSocketSender TCPlimelightPosController;

  public volatile double leftMotor1Voltage;
  public volatile double leftMotor2Voltage;
  public volatile double leftMotor3Voltage;
  public volatile double rightMotor1Voltage;
  public volatile double rightMotor2Voltage;
  public volatile double rightMotor3Voltage;

  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 40; //amps
  private final double TRIGGER_THRESHOLD_LIMIT = 0; //amp
  private final double TRIGGER_THRESHOLD_TIME = 0; //ms

  double runTime = Timer.getFPGATimestamp();


  /**
   * Default constructors for Drivetrain
   */
  private Drivetrain()
  {

    /**
     * Method which automaticallys allows us to switch between 6 wheel and 6 wheel
     * drive, and also allows us to switch between CAN vs PWM control on Drivetrain
     * 
     * 4 to 6 motors is controlled by boolean in RobotMap
     * 
     * CAN/PWM is controlled by jumper on MXP
     * 
     * Also allows us to detect comp chasis vs practice chassis and code for any
     * differences.
     */

      System.out.println("CAN Comp Bot Drivetrain enabled - 4 motors");
      _leftMotor1 = new TalonFX(RobotMap.DRIVETRAIN_LEFT_MOTOR_1_PDP);
      _leftMotor2 = new TalonFX(RobotMap.DRIVETRAIN_LEFT_MOTOR_2_PDP);
      _leftMotor3 = new TalonFX(RobotMap.DRIVETRAIN_LEFT_MOTOR_3_PDP);
      _rightMotor1 = new TalonFX(RobotMap.DRIVETRAIN_RIGHT_MOTOR_1_PDP);
      _rightMotor2 = new TalonFX(RobotMap.DRIVETRAIN_RIGHT_MOTOR_2_PDP);
      _rightMotor3 = new TalonFX(RobotMap.DRIVETRAIN_RIGHT_MOTOR_3_PDP);
    
      //speed limit 40

      talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
      CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

      _leftMotor1.configSupplyCurrentLimit(talonCurrentLimit);
      _leftMotor2.configSupplyCurrentLimit(talonCurrentLimit);
      _leftMotor3.configSupplyCurrentLimit(talonCurrentLimit);
      _rightMotor1.configSupplyCurrentLimit(talonCurrentLimit);
      _rightMotor2.configSupplyCurrentLimit(talonCurrentLimit);
      _rightMotor3.configSupplyCurrentLimit(talonCurrentLimit);
      
      //control frame every 20ms
      //_leftMotor1.setControlFramePeriodMs(20);
      //_leftMotor2.setControlFramePeriodMs(20);
      //_leftMotor3.setControlFramePeriodMs(20);
      //_rightMotor1.setControlFramePeriodMs(20);
      //_rightMotor2.setControlFramePeriodMs(20);
      //_rightMotor3.setControlFramePeriodMs(20);

      //status frame check thing every 500ms which is now broken because of the different motors
      //_leftMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus0,500);
      //_leftMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus0,500);
      //_leftMotor3.setPeriodicFramePeriod(PeriodicFrame.kStatus0,500);
      //_rightMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus0,500);
      //_rightMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus0,500);
      //_rightMotor3.setPeriodicFramePeriod(PeriodicFrame.kStatus0,500);

      //status frame check thing every 500ms which is now broken because of the different motors
      //_leftMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus1,500);
      //_leftMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus1,500);
      //_leftMotor3.setPeriodicFramePeriod(PeriodicFrame.kStatus1,500);
      //_rightMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus1,500);
      //_rightMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus1,500);
      //_rightMotor3.setPeriodicFramePeriod(PeriodicFrame.kStatus1,500);

      //status frame check thing every 500ms which is now broken because of the different motors
      //_leftMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus2,500);
      //_leftMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus2,500);
      //_leftMotor3.setPeriodicFramePeriod(PeriodicFrame.kStatus2,500);
      //_rightMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus2,500);
      //_rightMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus2,500);
      //_rightMotor3.setPeriodicFramePeriod(PeriodicFrame.kStatus2,500);

      // leftMotor1.setCANTimeout(100);
      // leftMotor3.setCANTimeout(100);
      // rightMotor2.setCANTimeout(100);null
      // rightMotor3.setCANTimeout(100);
    
    //how to call internal encoder
    _drivetrainRightEncoder = new AverageEncoder(
        RobotMap.RIGHT_DRIVE_ENCODER_A,
        RobotMap.RIGHT_DRIVE_ENCODER_B,
        RobotMap.DRIVE_ENCODER_PULSE_PER_ROT,
        RobotMap.DRIVE_ENCODER_DIST_PER_TICK,
        RobotMap.RIGHT_DRIVE_TRAIN_ENCODER_REVERSE,
        RobotMap.DRIVE_ENCODING_TYPE,
        RobotMap.DRIVE_SPEED_RETURN_TYPE,
        RobotMap.DRIVE_POS_RETURN_TYPE,
        RobotMap.DRIVE_AVG_ENCODER_VAL);

    _drivetrainLeftEncoder = new AverageEncoder(
        RobotMap.LEFT_DRIVE_ENCODER_A, 
        RobotMap.LEFT_DRIVE_ENCODER_B,
        RobotMap.DRIVE_ENCODER_PULSE_PER_ROT, 
        RobotMap.DRIVE_ENCODER_DIST_PER_TICK,
        RobotMap.LEFT_DRIVE_TRAIN_ENCODER_REVERSE, 
        RobotMap.DRIVE_ENCODING_TYPE,
        RobotMap.DRIVE_SPEED_RETURN_TYPE, 
        RobotMap.DRIVE_POS_RETURN_TYPE, 
        RobotMap.DRIVE_AVG_ENCODER_VAL);
    
    imu = new IMU(_drivetrainLeftEncoder, _drivetrainRightEncoder, RobotMap.WHEEL_BASE);

    limelight = new Limelight();
    limelight.setCamMode(1);
    limelight.setPipeline(7);

    // DriveStraight Controller
    // rotateController = new PIDPosition(
    //     "RotationController", 
    //     RobotMap.ROTATE_POSITION_P, 
    //     RobotMap.ROTATE_POSITION_I,
    //     RobotMap.ROTATE_POSITION_D, 
    //     _gyroSPI, 
    //     RobotMap.DRIVE_TRAIN_PID_PERIOD);

    
    // rotateDriveStraightController = new PIDPosition(
    //     "RotationStraightController",
    //     RobotMap.ROTATE_POSITION_P_Drive_Straight, 
    //     RobotMap.ROTATE_POSITION_I_Drive_Straight,
    //     RobotMap.ROTATE_POSITION_D_Drive_Straight, 
    //     _gyroSPI, 
    //     RobotMap.DRIVE_TRAIN_PID_PERIOD);

    // driveTrainPosController = new PIDPosition(
    //     "driveTrainPosController", 
    //     RobotMap.DRIVE_TRAIN_RIGHT_POSITION_P,
    //     RobotMap.DRIVE_TRAIN_RIGHT_POSITION_I, 
    //     RobotMap.DRIVE_TRAIN_RIGHT_POSITION_D, 
    //     imu,
    //     RobotMap.DRIVE_TRAIN_PID_PERIOD);

    // Spawn new PID Controller
    rightSpeedController = new PIDSpeed(
        "rightSpeedController", 
        RobotMap.DRIVE_TRAIN_RIGHT_SPEED_P,
        RobotMap.DRIVE_TRAIN_RIGHT_SPEED_I, 
        RobotMap.DRIVE_TRAIN_RIGHT_SPEED_D, 
        1, 
        _drivetrainRightEncoder,
        RobotMap.DRIVE_TRAIN_PID_PERIOD);

    leftSpeedController = new PIDSpeed(
        "leftSpeedController", 
        RobotMap.DRIVE_TRAIN_LEFT_SPEED_P,
        RobotMap.DRIVE_TRAIN_LEFT_SPEED_I, 
        RobotMap.DRIVE_TRAIN_LEFT_SPEED_D, 
        1, 
        _drivetrainLeftEncoder,
        RobotMap.DRIVE_TRAIN_PID_PERIOD);

      // Spawn new PID Controller
    // rightPosController = new PIDPosition(
    //     "rightPosController", 
    //     RobotMap.DRIVE_TRAIN_RIGHT_POSITION_P,
    //     RobotMap.DRIVE_TRAIN_RIGHT_POSITION_I, 
    //     RobotMap.DRIVE_TRAIN_RIGHT_POSITION_D, 
    //     1, 
    //     _drivetrainRightEncoder,
    //     RobotMap.DRIVE_TRAIN_PID_PERIOD);

    // leftPosController = new PIDPosition(
    //     "leftPosController", 
    //     RobotMap.DRIVE_TRAIN_LEFT_POSITION_P,
    //     RobotMap.DRIVE_TRAIN_LEFT_POSITION_I, 
    //     RobotMap.DRIVE_TRAIN_LEFT_POSITION_D, 
    //     1, 
    //     _drivetrainLeftEncoder,
    //     RobotMap.DRIVE_TRAIN_PID_PERIOD);

    
    //     // Limelight Controller
    // limelightPosController = new PIDPosition(
    //     "limelightPosController",
    //     RobotMap.LIMELIGHT_POSITION_P,
    //     RobotMap.LIMELIGHT_POSITION_I,
    //     RobotMap.LIMELIGHT_POSITION_D,
    //     limelight,
    //     RobotMap.DRIVE_TRAIN_PID_PERIOD);

    // add min and max output defaults and set array size
    rightSpeedController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
    leftSpeedController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
    rightPosController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
    leftPosController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
    driveTrainPosController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
    rotateController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
    rotateDriveStraightController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
    limelightPosController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);

    // start controller threads
    rightSpeedController.startThread();
    leftSpeedController.startThread();
    rightPosController.startThread();
    leftPosController.startThread();
    driveTrainPosController.startThread();
    rotateController.startThread();
    rotateDriveStraightController.startThread();
    limelightPosController.startThread();

    // start TCP Servers for DEBUGING ONLY
    TCPdrivePosController = new TCPSocketSender(RobotMap.TCP_SERVER_DRIVE_TRAIN_POS, driveTrainPosController);
    TCPdrivePosController.start();

    TCPrightSpeedController = new TCPSocketSender(RobotMap.TCO_SERVER_RIGHT_DRIVE_TRAIN_SPEED, rightSpeedController);
    TCPrightSpeedController.start();

    TCPleftSpeedController = new TCPSocketSender(RobotMap.TCP_SERVER_LEFT_DRIVE_TRAIN_SPEED, leftSpeedController);
    TCPleftSpeedController.start();

    TCPrightPosController = new TCPSocketSender(RobotMap.TCP_SERVER_RIGHT_DRIVE_TRAIN_POSITION, rightPosController);
    TCPrightPosController.start();

    TCPleftPosController = new TCPSocketSender(RobotMap.TCP_SERVER_LEFT_DRIVE_TRAIN_POSITION, leftPosController);
    TCPleftPosController.start();

    

    TCProtateController = new TCPSocketSender(RobotMap.TCP_SERVER_ROTATE_CONTROLLER, rotateController);
    TCProtateController.start();

    TCProtateController = new TCPSocketSender(RobotMap.TCP_SERVER_ROTATE_CONTROLLER_STRAIGHT,rotateDriveStraightController);
    TCProtateController.start();

    TCPlimelightPosController = new TCPSocketSender(RobotMap.TCP_SERVER_ROTATE_CONTROLLER_WITH_CAMERA,limelightPosController);
    TCPlimelightPosController.start();

    leftMotor1Voltage = 0;
    leftMotor2Voltage = 0;
    leftMotor3Voltage = 0;

    rightMotor1Voltage = 0;
    rightMotor2Voltage = 0;
    rightMotor3Voltage = 0;
    

    // Log sensor data
    ConsolePrinter.putNumber("DTRight1MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR_1_PDP);}, true, false);
    ConsolePrinter.putNumber("DTRight2MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR_2_PDP);}, true, false);
    ConsolePrinter.putNumber("DTRight3MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR_3_PDP);}, true, false);
    ConsolePrinter.putNumber("DTLeft1MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR_1_PDP);}, true, false);
    ConsolePrinter.putNumber("DTLeft2MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR_2_PDP);}, true, false);
    ConsolePrinter.putNumber("DTLeft3MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR_3_PDP);}, true, false);
    
  }

  /**
   * Calls instance object and makes it a singleton object of type Drivetrain
   * 
   * @returns Drivetrain object "instance"
   */
  public static Drivetrain getInstance()
  {
    if (instance == null)
      instance = new Drivetrain();

    return instance;
  }

  /**
   * Calls left motor 1 and creates a local variable "speed" Refers to boolean in
   * Robot map and if true, speed = - speed Uses set() command to assign the new
   * speed to left motor 1
   * 
   * @param double speed between -1 and 1 negative is reverse, positive if
   *        forward, 0 is stationary
   */
  private void driveleftMotor1(double speed)
  {
    if (RobotMap.DT_REVERSE_LEFT1)
      speed = -speed;

    _leftMotor1.set(ControlMode.PercentOutput, speed);
    leftMotor1Voltage = Robot.pdp.getBatteryVoltage() * speed;

  }

  /**
   * Calls left motor 2 and creates a local variable "speed" Refers to boolean in
   * Robot map and if true, speed = - speed Uses set() command to assign the new
   * speed to left motor 2
   * 
   * @param double speed between -1 and 1 negative is reverse, positive if
   *        forward, 0 is stationary
   */
  private void driveleftMotor2(double speed)
  {
    if (RobotMap.DT_REVERSE_LEFT2)
      speed = -speed;

    _leftMotor2.set(ControlMode.PercentOutput, speed);
    leftMotor2Voltage = Robot.pdp.getBatteryVoltage() * speed;
  }

  private void driveleftMotor3(double speed)
  {
    if (RobotMap.DT_REVERSE_LEFT3)
      speed = -speed;

    _leftMotor3.set(ControlMode.PercentOutput, speed);
    leftMotor3Voltage = Robot.pdp.getBatteryVoltage() * speed;
  }

  /**
   * Take in double speed and sets it to left motors 1, 2, and 3
   * 
   * @param speed is a double between -1 and 1 negative is reverse, positive if
   *              forward, 0 is stationary
   */
  public void driveLeft(double speed)
  {
    if (RobotMap.DT_3_MOTORS_PER_SIDE)
    {
      driveleftMotor1(speed);
      driveleftMotor2(speed);
      driveleftMotor3(speed);
    }
    else
    {
      driveleftMotor1(speed);
      driveleftMotor2(speed);
    }
  }
/**
   * Calls right motor 1 and creates a local variable "speed" Refers to boolean in
   * Robot map and if true, speed = - speed Uses set() command to assign the new
   * speed to right motor 1
   * 
   * @param double speed between -1 and 1 negative is reverse, positive if
   *        forward, 0 is stationary
   */
  private void driverightMotor1(double speed)
  {
    if (RobotMap.DT_REVERSE_RIGHT1)
      speed = -speed;

    _rightMotor1.set(ControlMode.PercentOutput, speed);
    rightMotor1Voltage = Robot.pdp.getBatteryVoltage() * speed;
  }

  /**
   * Calls right motor 2 and creates a local variable "speed" Refers to boolean in
   * Robot map and if true, speed = - speed Uses set() command to assign the new
   * speed to right motor 2
   * 
   * @param double speed between -1 and 1 negative is reverse, positive if
   *        forward, 0 is stationary
   */
  private void driverightMotor2(double speed)
  {
    if (RobotMap.DT_REVERSE_RIGHT2)
      speed = -speed;

    _rightMotor2.set(ControlMode.PercentOutput, speed);
    rightMotor2Voltage = Robot.pdp.getBatteryVoltage() * speed;
  }

  /**
   * Calls right motor 3 and creates a local variable "speed" Refers to boolean in
   * Robot map and if true, speed = - speed Uses set() command to assign the new
   * speed to right motor 3
   * 
   * @param double speed between -1 and 1 negative is reverse, positive if
   *        forward, 0 is stationary
   */
  private void driverightMotor3(double speed)
  {
    if (RobotMap.DT_REVERSE_RIGHT3)
      speed = -speed;

    _rightMotor3.set(ControlMode.PercentOutput, speed);
    rightMotor3Voltage = Robot.pdp.getBatteryVoltage() * speed;
  }
  
  private void driveRight(double speed)
  {
    if(RobotMap.DT_3_MOTORS_PER_SIDE)
    {
      driverightMotor1(speed);
      driverightMotor2(speed);
      driverightMotor3(speed);
    }
    else
    driverightMotor1(speed);
    driverightMotor2(speed);
  }

  public void tankDrive(double leftSpeed, double rightSpeed)
  {
    driveLeft(leftSpeed);
    driveRight(rightSpeed);

  }


  @Override
  protected void initDefaultCommand() {

  }

}