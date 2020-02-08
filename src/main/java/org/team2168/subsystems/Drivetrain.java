package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import org.team2168.Constants;
import org.team2168.RobotMap;
import org.team2168.PID.controllers.PIDPosition;
import org.team2168.PID.sensors.Limelight;
import org.team2168.commands.drivetrain.DriveWithJoystick;
import org.team2168.utils.TCPSocketSender;

import edu.wpi.first.wpilibj.command.Subsystem;


public class Drivetrain extends Subsystem {  
  private static TalonFX _leftMotor1;
  private static TalonFX _leftMotor2;
  private static TalonFX _leftMotor3;
  private static TalonFX _rightMotor1;
  private static TalonFX _rightMotor2;
  private static TalonFX _rightMotor3;
  private static PigeonIMU _pidgey;
  
  public Limelight limelight;
  public PIDPosition limelightPosController;

  TCPSocketSender TCPlimelightPosController;

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

  	/** Invert Directions for Left and Right */
	TalonFXInvertType _leftInvert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
	TalonFXInvertType _rightInvert = TalonFXInvertType.Clockwise; //Same as invert = "true"

	/** Config Objects for motor controllers */
	TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
  TalonFXConfiguration _rightConfig = new TalonFXConfiguration();
  
	private static final double TICKS_PER_REV = 2048.0; //one event per edge on each quadrature channel
	private static final double TICKS_PER_100MS = TICKS_PER_REV / 10.0;
	private static final double GEAR_RATIO = (50.0/10.0) * (40.0/22.0);
	private static final double WHEEL_DIAMETER = 6.0; //inches
	private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; //inches
	private static final double PIGEON_UNITS_PER_ROTATION = 8192.0;;
	private static final double DEGREES_PER_REV = 360.0;
	private static final double PIGEON_UNITS_PER_DEGREE = PIGEON_UNITS_PER_ROTATION/360;
  private static final double WHEEL_BASE = 24.0; //distance between wheels (width) in inches
  
  private double setPointPosition_sensorUnits;
  private double setPointHeading_sensorUnits;
  private double setPointVelocity_sensorUnits;

  private static final double CRUISE_VEL_STRAIGHT = 10.0*12.0;
  private static final double MAX_ACC_STRAIGHT = 10.0*12.0;

  private static final double CRUISE_VEL_TURN = 10.0*12.0;
  private static final double MAX_ACC_TURN = 5.0*12.0;



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
      _pidgey = new PigeonIMU(17);

    
      talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
      CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

      _leftMotor1.configSupplyCurrentLimit(talonCurrentLimit);
      _leftMotor2.configSupplyCurrentLimit(talonCurrentLimit);
      _leftMotor3.configSupplyCurrentLimit(talonCurrentLimit);
      _rightMotor1.configSupplyCurrentLimit(talonCurrentLimit);
      _rightMotor2.configSupplyCurrentLimit(talonCurrentLimit);
      _rightMotor3.configSupplyCurrentLimit(talonCurrentLimit);

    //values TBD
    limelight = new Limelight();
    limelight.setCamMode(0);
    limelight.setPipeline(4);

    // Log sensor data
    // ConsolePrinter.putNumber("DTRight1MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR_1_PDP);}, true, false);
    // ConsolePrinter.putNumber("DTRight2MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR_2_PDP);}, true, false);
    // ConsolePrinter.putNumber("DTRight3MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR_3_PDP);}, true, false);
    // ConsolePrinter.putNumber("DTLeft1MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR_1_PDP);}, true, false);
    // ConsolePrinter.putNumber("DTLeft2MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR_2_PDP);}, true, false);
    // ConsolePrinter.putNumber("DTLeft3MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR_3_PDP);}, true, false);
    
    // limelightPosController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
    // limelightPosController.startThread();

    // TCPlimelightPosController = new TCPSocketSender(RobotMap.TCP_SERVER_ROTATE_CONTROLLER_WITH_CAMERA,limelightPosController);
    // TCPlimelightPosController.start();
    
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

  public double getPosition()
  {
    return ticks_to_inches(_leftMotor1.getSelectedSensorPosition(Constants.PID_PRIMARY));
  }

  public double getVelocity()
  {
    return ticks_per_100ms_to_inches_per_sec(_leftMotor1.getSelectedSensorVelocity(Constants.PID_PRIMARY));
  }

  public double getHeading()
  {
    //return _pidgey.getFusedHeading();
    return ticks_to_degrees(_rightMotor1.getSelectedSensorPosition(Constants.PID_TURN));
  }

  public void setSetPointPosition(double setPoint, double setAngle) {
    setPointPosition_sensorUnits = inches_to_ticks(setPoint);
    setPointHeading_sensorUnits = degrees_to_ticks(setAngle);

    _rightMotor1.set(ControlMode.MotionMagic, setPointPosition_sensorUnits, DemandType.AuxPID, setPointHeading_sensorUnits);
    _rightMotor2.follow(_rightMotor1, FollowerType.PercentOutput);
    _rightMotor3.follow(_rightMotor1, FollowerType.PercentOutput);
    _leftMotor1.follow(_rightMotor1, FollowerType.AuxOutput1);
    _leftMotor2.follow(_rightMotor1, FollowerType.AuxOutput1);
    _leftMotor3.follow(_rightMotor1, FollowerType.AuxOutput1);
  }

  public void setSetPointVelocity(double setPoint, double setAngle)
  {
    setPointVelocity_sensorUnits = inches_per_sec_to_ticks_per_100ms(setPoint);
    setPointHeading_sensorUnits = degrees_to_ticks(setAngle);

    _rightMotor1.set(ControlMode.Velocity, setPointVelocity_sensorUnits, DemandType.AuxPID, setPointHeading_sensorUnits);
    _rightMotor2.follow(_rightMotor1, FollowerType.PercentOutput);
    _rightMotor3.follow(_rightMotor1, FollowerType.PercentOutput);
    _leftMotor1.follow(_rightMotor1, FollowerType.AuxOutput1);
    _leftMotor2.follow(_rightMotor1, FollowerType.AuxOutput1);
    _leftMotor3.follow(_rightMotor1, FollowerType.AuxOutput1);
  }

  public void changeMaxVelAcc(boolean straightmode)
  {
    if(straightmode) {
      /* Motion Magic Configs */
      _rightMotor1.configMotionAcceleration((int) (inches_per_sec_to_ticks_per_100ms(MAX_ACC_STRAIGHT))); //(distance units per 100 ms) per second 
      _rightMotor1.configMotionCruiseVelocity((int) (inches_per_sec_to_ticks_per_100ms(CRUISE_VEL_STRAIGHT))); //distance units per 100 ms

    }
    else {
      /* Motion Magic Configs */
      _rightMotor1.configMotionAcceleration((int) (inches_per_sec_to_ticks_per_100ms(MAX_ACC_TURN))); //(distance units per 100 ms) per second 
      _rightMotor1.configMotionCruiseVelocity((int) (inches_per_sec_to_ticks_per_100ms(CRUISE_VEL_TURN))); //distance units per 100 ms

    }
  }

  public double getMaxVel() {
    return inches_per_sec_to_ticks_per_100ms(10.0);
    // return CRUISE_VEL_STRAIGHT; 
  }

  public double getErrorPosition() {
    return ticks_to_inches(setPointPosition_sensorUnits-_rightMotor1.getSelectedSensorPosition(Constants.PID_PRIMARY));
    //return _leftMotor1.getClosedLoopError(kPIDLoopIdx)/TICKS_PER_REV;--only for nonMotionMagic or nonMotion Profile
  }

  public double getErrorHeading() {
    return ticks_to_degrees(setPointHeading_sensorUnits-_rightMotor1.getSelectedSensorPosition(Constants.PID_TURN)); 
  }

  public double getErrorVelocity() {
    return ticks_per_100ms_to_inches_per_sec(setPointVelocity_sensorUnits-_rightMotor1.getSelectedSensorVelocity(Constants.PID_PRIMARY));
  }

  public double getSetPointPosition()
  {
    return this.setPointPosition_sensorUnits;
  }

  public double getSetPointHeading()
  {
    return this.setPointHeading_sensorUnits;
  }

  public double getSetPointVelocity()
  {
    return this.setPointVelocity_sensorUnits;
  }

    /** Zero all sensors, both Talons and Pigeon */
  public void zeroSensors() {
    _leftMotor1.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
    _rightMotor1.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
    _pidgey.setYaw(0, Constants.kTimeoutMs);
    _pidgey.setAccumZAngle(0, Constants.kTimeoutMs);
    System.out.println("[Quadrature Encoders + Pigeon] All sensors are zeroed.\n");
  }

  /** Zero QuadEncoders, used to reset position when initializing Motion Magic */
  public void zeroDistance(){
    _leftMotor1.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
    _rightMotor1.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
    System.out.println("[Quadrature Encoders] All encoders are zeroed.\n");
  }

  public void zeroPigeon() {
    _pidgey.setYaw(0, Constants.kTimeoutMs);
    _pidgey.setAccumZAngle(0, Constants.kTimeoutMs);
  }

  /** 
   * Determines if SensorSum or SensorDiff should be used 
   * for combining left/right sensors into Robot Distance.  
   * 
   * Assumes Aux Position is set as Remote Sensor 0.  
   * 
   * configAllSettings must still be called on the master config
   * after this function modifies the config values. 
   * 
   * @param masterInvertType Invert of the Master Talon
   * @param masterConfig Configuration object to fill
   */
    private void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig){
    /**
     * Determine if we need a Sum or Difference.
     * 
     * The auxiliary Talon FX will always be positive
     * in the forward direction because it's a selected sensor
     * over the CAN bus.
     * 
     * The master's native integrated sensor may not always be positive when forward because
     * sensor phase is only applied to *Selected Sensors*, not native
     * sensor sources.  And we need the native to be combined with the 
     * aux (other side's) distance into a single robot distance.
     */

    /* THIS FUNCTION should not need to be modified. 
        This setup will work regardless of whether the master
        is on the Right or Left side since it only deals with
        distance magnitude.  */

    /* Check if we're inverted */
    if (masterInvertType == TalonFXInvertType.Clockwise){
      /* 
        If master is inverted, that means the integrated sensor
        will be negative in the forward direction.
        If master is inverted, the final sum/diff result will also be inverted.
        This is how Talon FX corrects the sensor phase when inverting 
        the motor direction.  This inversion applies to the *Selected Sensor*,
        not the native value.
        Will a sensor sum or difference give us a positive total magnitude?
        Remember the Master is one side of your drivetrain distance and 
        Auxiliary is the other side's distance.
          Phase | Term 0   |   Term 1  | Result
        Sum:  -1 *((-)Master + (+)Aux   )| NOT OK, will cancel each other out
        Diff: -1 *((-)Master - (+)Aux   )| OK - This is what we want, magnitude will be correct and positive.
        Diff: -1 *((+)Aux    - (-)Master)| NOT OK, magnitude will be correct but negative
      */

      masterConfig.diff0Term = FeedbackDevice.IntegratedSensor; //Local Integrated Sensor
      masterConfig.diff1Term = FeedbackDevice.RemoteSensor0;   //Aux Selected Sensor
      masterConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.SensorDifference; //Diff0 - Diff1
    } else {
      /* Master is not inverted, both sides are positive so we can sum them. */
      masterConfig.sum0Term = FeedbackDevice.RemoteSensor0;    //Aux Selected Sensor
      masterConfig.sum1Term = FeedbackDevice.IntegratedSensor; //Local IntegratedSensor
      masterConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.SensorSum; //Sum0 + Sum1
    }

    /* Since the Distance is the sum of the two sides, divide by 2 so the total isn't double
        the real-world value */
    masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
    }


  /**
   * Converts a setpoint in degrees to IMU 'encoder ticks'
   * @param setpoint
   * @return
   */
  private double degrees_to_ticks(double setpoint) {
    return (setpoint / DEGREES_PER_REV) * PIGEON_UNITS_PER_ROTATION / 2.0;
  }

  private double ticks_to_degrees(double setpoint) {
    return (setpoint / PIGEON_UNITS_PER_ROTATION) * DEGREES_PER_REV * 2.0; 
  }

  private double degrees_per_sec_to_ticks_per_100ms(double setpoint) {
    return (degrees_to_ticks(setpoint) / 10.0);
  }

  private double ticks_per_100ms_to_degrees_per_sec(double setpoint) {
    return (ticks_to_degrees(setpoint) * 10.0);
  }

  private double inches_to_ticks(double setpoint) {
    return (setpoint * TICKS_PER_REV * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;
  }

  private double ticks_to_inches(double setpoint) {
    return (setpoint * WHEEL_CIRCUMFERENCE) / (TICKS_PER_REV * GEAR_RATIO);
  }

  private double inches_per_sec_to_ticks_per_100ms(double setpoint) {
    return inches_to_ticks(setpoint) / 10.0;
  }

  private double ticks_per_100ms_to_inches_per_sec(double setpoint) {
    return ticks_to_inches(setpoint) * 10.0;
  }

  private double revs_to_ticks(double revs) {
    return revs * (TICKS_PER_REV * GEAR_RATIO);
  }

  private double degrees_to_wheel_revs (double degrees) {
    return (degrees / 360.0) * (WHEEL_BASE / WHEEL_DIAMETER);
  }


  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoystick());
  }

}