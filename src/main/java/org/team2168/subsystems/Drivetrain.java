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
import org.team2168.commands.drivetrain.DriveWithJoystick;

import edu.wpi.first.wpilibj.command.Subsystem;


public class Drivetrain extends Subsystem {  
  private static TalonFX _leftMotor1;
  private static TalonFX _leftMotor2;
  private static TalonFX _leftMotor3;
  private static TalonFX _rightMotor1;
  private static TalonFX _rightMotor2;
  private static TalonFX _rightMotor3;
  private static PigeonIMU _pidgey;

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

      _leftMotor1.setNeutralMode(NeutralMode.Brake);
      _leftMotor2.setNeutralMode(NeutralMode.Coast);
      _leftMotor3.setNeutralMode(NeutralMode.Coast);
      _rightMotor1.setNeutralMode(NeutralMode.Brake);
      _rightMotor2.setNeutralMode(NeutralMode.Coast);
      _rightMotor3.setNeutralMode(NeutralMode.Coast);

          /* Configure output and sensor direction */
    _leftMotor1.setInverted(_leftInvert);
    _leftMotor2.setInverted(_leftInvert);
    _leftMotor3.setInverted(_leftInvert);
    _rightMotor1.setInverted(_rightInvert);
    _rightMotor2.setInverted(_rightInvert);
    _rightMotor3.setInverted(_rightInvert);

    /* Reset Pigeon Configs */
    _pidgey.configFactoryDefault();
    
        /** Feedback Sensor Configuration */

    /** Distance Configs */

    /* Configure the left Talon's selected sensor as integrated sensor */
    _leftConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor; //Local Feedback Source

    /* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
    _rightConfig.remoteFilter0.remoteSensorDeviceID = _leftMotor1.getDeviceID(); //Device ID of Remote Source
    _rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type
    
    /* Now that the Left sensor can be used by the master Talon,
      * set up the Left (Aux) and Right (Master) distance into a single
      * Robot distance as the Master's Selected Sensor 0. */
    setRobotDistanceConfigs(_rightInvert, _rightConfig);

    /* FPID for Distance */
    _rightConfig.slot0.kF = Constants.kGains_Distance.kF;
    _rightConfig.slot0.kP = Constants.kGains_Distance.kP;
    _rightConfig.slot0.kI = Constants.kGains_Distance.kI;
    _rightConfig.slot0.kD = Constants.kGains_Distance.kD;
    _rightConfig.slot0.integralZone = Constants.kGains_Distance.kIzone;
    _rightConfig.slot0.closedLoopPeakOutput = Constants.kGains_Distance.kPeakOutput;



    /** Heading Configs */
    _rightConfig.remoteFilter1.remoteSensorDeviceID = _pidgey.getDeviceID();    //Pigeon Device ID
    _rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw; //This is for a Pigeon over CAN
    _rightConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor1; //Set as the Aux Sensor
    _rightConfig.auxiliaryPID.selectedFeedbackCoefficient = 3600.0 / Constants.kPigeonUnitsPerRotation; //Convert Yaw to tenths of a degree

    /* false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
      *   This is typical when the master is the right Talon FX and using Pigeon
      * 
      * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
      *   This is typical when the master is the left Talon FX and using Pigeon
      */
    _rightConfig.auxPIDPolarity = false;

    /* FPID for Heading */
    _rightConfig.slot1.kF = Constants.kGains_Turning.kF;
    _rightConfig.slot1.kP = Constants.kGains_Turning.kP;
    _rightConfig.slot1.kI = Constants.kGains_Turning.kI;
    _rightConfig.slot1.kD = Constants.kGains_Turning.kD;
    _rightConfig.slot1.integralZone = Constants.kGains_Turning.kIzone;
    _rightConfig.slot1.closedLoopPeakOutput = Constants.kGains_Turning.kPeakOutput;


    /* Config the neutral deadband. */
    _leftConfig.neutralDeadband = Constants.kNeutralDeadband;
    _rightConfig.neutralDeadband = Constants.kNeutralDeadband;


    /**
     * 1ms per loop.  PID loop can be slowed down if need be.
     * For example,
     * - if sensor updates are too slow
     * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
     * - sensor movement is very slow causing the derivative error to be near zero.
     */
    int closedLoopTimeMs = 1;
    _rightMotor1.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
    _rightMotor1.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

    /* Motion Magic Configs */
    _rightConfig.motionAcceleration = (int) (inches_per_sec_to_ticks_per_100ms(5.0*12.0)); //(distance units per 100 ms) per second //7500
    _rightConfig.motionCruiseVelocity = (int) (inches_per_sec_to_ticks_per_100ms(10.0*12.0)); //distance units per 100 ms //10000



    /* APPLY the config settings */
    _leftMotor1.configAllSettings(_leftConfig);
    _rightMotor1.configAllSettings(_rightConfig);

    _rightMotor1.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
    _rightMotor1.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);


    /* Set status frame periods to ensure we don't have stale data */
    /* These aren't configs (they're not persistant) so we can set these after the configs.  */
    _rightMotor1.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
    _rightMotor1.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
    _rightMotor1.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
    _rightMotor1.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, Constants.kTimeoutMs);
    _leftMotor1.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);
    _pidgey.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, Constants.kTimeoutMs);

    // Log sensor data
    // ConsolePrinter.putNumber("DTRight1MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR_1_PDP);}, true, false);
    // ConsolePrinter.putNumber("DTRight2MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR_2_PDP);}, true, false);
    // ConsolePrinter.putNumber("DTRight3MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR_3_PDP);}, true, false);
    // ConsolePrinter.putNumber("DTLeft1MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR_1_PDP);}, true, false);
    // ConsolePrinter.putNumber("DTLeft2MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR_2_PDP);}, true, false);
    // ConsolePrinter.putNumber("DTLeft3MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR_3_PDP);}, true, false);
    
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

  public void changeMaxVelAcc(boolean straightmode)
  {
    if(straightmode) {
      /* Motion Magic Configs */
      _rightMotor1.configMotionAcceleration((int) (inches_per_sec_to_ticks_per_100ms(10.0*12.0))); //(distance units per 100 ms) per second 
      _rightMotor1.configMotionCruiseVelocity((int) (inches_per_sec_to_ticks_per_100ms(10.0*12.0))); //distance units per 100 ms

    }
    else {
      /* Motion Magic Configs */
      _rightMotor1.configMotionAcceleration((int) (inches_per_sec_to_ticks_per_100ms(5.0*12.0))); //(distance units per 100 ms) per second 
      _rightMotor1.configMotionCruiseVelocity((int) (inches_per_sec_to_ticks_per_100ms(10.0*12.0))); //distance units per 100 ms

    }


  }

  public double getErrorPosition() {
    return ticks_to_inches(setPointPosition_sensorUnits-_rightMotor1.getSelectedSensorPosition(Constants.PID_PRIMARY));
    //return _leftMotor1.getClosedLoopError(kPIDLoopIdx)/TICKS_PER_REV;--only for nonMotionMagic or nonMotion Profile
  }

  public double getErrorHeading() {
    return ticks_to_degrees(setPointHeading_sensorUnits-_rightMotor1.getSelectedSensorPosition(Constants.PID_TURN)); 
  }

  public double getSetPointPosition()
  {
    return this.setPointPosition_sensorUnits;
  }

  public double getSetPointHeading()
  {
    return this.setPointHeading_sensorUnits;
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