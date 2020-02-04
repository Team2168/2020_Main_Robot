package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import org.team2168.Gains;
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
  private static final boolean ENABLE_CURRENT_LIMIT = true;
  private static final double CONTINUOUS_CURRENT_LIMIT = 40; //amps
  private static final double TRIGGER_THRESHOLD_LIMIT = 60; //amp
  private static final double TRIGGER_THRESHOLD_TIME = 200; //ms
  private static final boolean DT_REVERSE_LEFT = false;
  private static final boolean DT_REVERSE_RIGHT = true; 
  private static final boolean DT_SENSOR_PHASE_LEFT = false;
  private static final boolean DT_SENSOR_PHASE_RIGHT = true;
  private static final boolean DT_3_MOTORS_PER_SIDE = true;

  /** ---- Flat constants, you should not need to change these ---- */
  /* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
  private static final int REMOTE_0 = 0;
  private static final int REMOTE_1 = 1;
  /* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
  private static final int PID_PRIMARY = 0;
  private static final int PID_TURN = 1;
  /* Firmware currently supports slots [0, 3] and can be used for either PID Set */
  private static final int SLOT_0 = 0;
  private static final int SLOT_1 = 1;
  // private static final int SLOT_2 = 2;
  // private static final int SLOT_3 = 3;
  /* ---- Named slots, used to clarify code ---- */
  private static final int kSlot_Distance = SLOT_0;
  private static final int kSlot_Turning = SLOT_1;
  // private static final int kSlot_Velocity = SLOT_2;
  // private static final int kSlot_MotProf = SLOT_3; 

  /**
   * set to zero to skip waiting for confirmation, set to nonzero to wait and
   * report to DS if action fails.
   */
  private static final int kTimeoutMs = 30;

  /**
   * Motor neutral dead-band, set to the minimum 0.1%.
   */
  private static final double kNeutralDeadband = 0.001;
  // private static final double MAX_VELOCITY = 11000;

  /**
   * PID Gains may have to be adjusted based on the responsiveness of control loop.
   * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
   * Not all set of Gains are used in this project and may be removed as desired.
   * 
   * 	                                                    kP   kI   kD   kF                    Iz    PeakOut */
  private static final Gains kGains_Distance = new Gains( 0.2, 0.0,  0.0, 0.0,                 100,  0.50 );
  private static final Gains kGains_Turning = new Gains( 2.0, 0.0,  4.0, 0.0,                  200,  0.60 );
  // private static final Gains kGains_Velocity = new Gains( 0.1, 0.0, 20.0, 1023.0/MAX_VELOCITY, 300,  0.50 );
  // private static final Gains kGains_MotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/MAX_VELOCITY,  400,  1.00 );
  /**
   * Convert target RPM to ticks / 100ms.
   * velocity setpoint is in units/100ms
   */
  private static final double TICKS_PER_REV = 2048.0; //one event per edge on each quadrature channel
  private static final double TICKS_PER_100MS = TICKS_PER_REV / 10.0;
  private static final double GEAR_RATIO = (50.0/10.0) * (40.0/22.0);
  private static final double WHEEL_CIRCUMFERENCE = 6.0 * Math.PI;
  private static final double TICKS_PER_INCH = TICKS_PER_REV * GEAR_RATIO / WHEEL_CIRCUMFERENCE;
  private static final double TICKS_PER_INCH_PER_100MS = TICKS_PER_100MS * GEAR_RATIO / WHEEL_CIRCUMFERENCE;
  /**
   * This is a property of the Pigeon IMU, and should not be changed.
   */
  private static final int PIGEON_UNITS_PER_ROTATION = 8192;
  private static final double PIGEON_UNITS_PER_DEGREE = PIGEON_UNITS_PER_ROTATION/360;

  /**
   * Default constructors for Drivetrain
   */
  private Drivetrain() {
    //System.out.println("CAN Comp Bot Drivetrain enabled - 6 motors");
    _leftMotor1 = new TalonFX(RobotMap.DRIVETRAIN_LEFT_MOTOR_1_PDP);
    _leftMotor2 = new TalonFX(RobotMap.DRIVETRAIN_LEFT_MOTOR_2_PDP);
    _leftMotor3 = new TalonFX(RobotMap.DRIVETRAIN_LEFT_MOTOR_3_PDP);
    _rightMotor1 = new TalonFX(RobotMap.DRIVETRAIN_RIGHT_MOTOR_1_PDP);
    _rightMotor2 = new TalonFX(RobotMap.DRIVETRAIN_RIGHT_MOTOR_2_PDP);
    _rightMotor3 = new TalonFX(RobotMap.DRIVETRAIN_RIGHT_MOTOR_3_PDP);

    _pidgey = new PigeonIMU(RobotMap.DRIVETRAIN_PIGEON_CAN_ID);

    /* Factory Default all hardware to prevent unexpected behaviour */
    _leftMotor1.configFactoryDefault();
    _leftMotor2.configFactoryDefault();
    _leftMotor3.configFactoryDefault();
    _rightMotor1.configFactoryDefault();
    _rightMotor2.configFactoryDefault();
    _rightMotor3.configFactoryDefault();
    _pidgey.configFactoryDefault();
  
    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
    CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    _leftMotor1.configSupplyCurrentLimit(talonCurrentLimit);
    _leftMotor2.configSupplyCurrentLimit(talonCurrentLimit);
    _leftMotor3.configSupplyCurrentLimit(talonCurrentLimit);
    _rightMotor1.configSupplyCurrentLimit(talonCurrentLimit);
    _rightMotor2.configSupplyCurrentLimit(talonCurrentLimit);
    _rightMotor3.configSupplyCurrentLimit(talonCurrentLimit);

    /* Set Neutral Mode */
    _leftMotor1.setNeutralMode(NeutralMode.Brake);
    _leftMotor2.setNeutralMode(NeutralMode.Coast);
    _leftMotor3.setNeutralMode(NeutralMode.Coast);
    _rightMotor1.setNeutralMode(NeutralMode.Brake);
    _rightMotor2.setNeutralMode(NeutralMode.Coast);
    _rightMotor3.setNeutralMode(NeutralMode.Coast);

    /** Feedback Sensor Configuration */

    /* Configure the left Talon's selected sensor as integrated sensor */
		_leftMotor1.configSelectedFeedbackSensor(	FeedbackDevice.IntegratedSensor,  // Local Feedback Source
													PID_PRIMARY,          // PID Slot for Source [0, 1]
													kTimeoutMs);                    // Configuration Timeout

		/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		_rightMotor1.configRemoteFeedbackFilter(_leftMotor1.getDeviceID(),  // Device ID of Source
												RemoteSensorSource.TalonSRX_SelectedSensor,	    // Remote Feedback Source
												REMOTE_0,                             // Source number [0, 1]
												kTimeoutMs);                                    // Configuration Timeout
		
		/* Configure the Pigeon IMU to the other remote slot available on the right Talon */
		_rightMotor1.configRemoteFeedbackFilter(_pidgey.getDeviceID(),
												RemoteSensorSource.Pigeon_Yaw,
												REMOTE_1,	
												kTimeoutMs);
		
		/* Setup Sum signal to be used for Distance */
		_rightMotor1.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.RemoteSensor0, kTimeoutMs);    // Feedback Device of Remote Talon
		_rightMotor1.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.IntegratedSensor, kTimeoutMs);	// Quadrature Encoder of current Talon
		
		/* Configure Sum [Difference of both internal encoders] to be used for Primary PID Index */
		_rightMotor1.configSelectedFeedbackSensor(	FeedbackDevice.SensorDifference, 
													PID_PRIMARY,
													kTimeoutMs);
		
		/* Scale Feedback by 0.5 to half the sum of Distance */ //not working--do this within the setpoint
		_rightMotor1.configSelectedFeedbackCoefficient(	1,      // Coefficient
														PID_PRIMARY,          // PID Slot of Source 
														kTimeoutMs);                    // Configuration Timeout
		
		/* Configure Remote 1 [Pigeon IMU's Yaw] to be used for Auxiliary PID Index */
		_rightMotor1.configSelectedFeedbackSensor(	FeedbackDevice.RemoteSensor1,
													PID_TURN,
													kTimeoutMs);
		
		/* Scale the Feedback Sensor using a coefficient */
		_rightMotor1.configSelectedFeedbackCoefficient(	1,
														PID_TURN,
														kTimeoutMs);
		
		/* Configure output and sensor direction */
    _leftMotor1.setInverted(DT_REVERSE_LEFT);
    _leftMotor2.setInverted(DT_REVERSE_LEFT);
    _leftMotor3.setInverted(DT_REVERSE_LEFT);
    _leftMotor1.setSensorPhase(DT_SENSOR_PHASE_LEFT);
    _leftMotor2.setSensorPhase(DT_SENSOR_PHASE_LEFT);
    _leftMotor3.setSensorPhase(DT_SENSOR_PHASE_LEFT);
    _rightMotor1.setInverted(DT_REVERSE_RIGHT);
    _rightMotor2.setInverted(DT_REVERSE_RIGHT);
    _rightMotor3.setInverted(DT_REVERSE_RIGHT);
    _rightMotor1.setSensorPhase(DT_SENSOR_PHASE_RIGHT);
    _rightMotor2.setSensorPhase(DT_SENSOR_PHASE_RIGHT);
    _rightMotor3.setSensorPhase(DT_SENSOR_PHASE_RIGHT);
		
		/* Set status frame periods to ensure we don't have stale data */
		_rightMotor1.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, kTimeoutMs);
		_rightMotor1.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, kTimeoutMs);
		_rightMotor1.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, kTimeoutMs);
		_rightMotor1.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, kTimeoutMs);
		_leftMotor1.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);
		_pidgey.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, kTimeoutMs);

		/* Configure neutral deadband */
    _rightMotor1.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);
    _rightMotor2.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);
    _rightMotor3.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);
    _leftMotor1.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);
    _leftMotor2.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);
    _leftMotor3.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);
		
		/* Motion Magic Configurations */
		_rightMotor1.configMotionAcceleration((int) (10*12*TICKS_PER_INCH_PER_100MS*2), kTimeoutMs); //should be inches per sec
		_rightMotor1.configMotionCruiseVelocity((int) (10*12*TICKS_PER_INCH_PER_100MS*2), kTimeoutMs);

		/**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_leftMotor1.configPeakOutputForward(+1.0, kTimeoutMs);
    _leftMotor1.configPeakOutputReverse(-1.0, kTimeoutMs);
    _leftMotor2.configPeakOutputForward(+1.0, kTimeoutMs);
    _leftMotor2.configPeakOutputReverse(-1.0, kTimeoutMs);
    _leftMotor3.configPeakOutputForward(+1.0, kTimeoutMs);
		_leftMotor3.configPeakOutputReverse(-1.0, kTimeoutMs);
		_rightMotor1.configPeakOutputForward(+1.0, kTimeoutMs);
    _rightMotor1.configPeakOutputReverse(-1.0, kTimeoutMs);
    _rightMotor2.configPeakOutputForward(+1.0, kTimeoutMs);
    _rightMotor2.configPeakOutputReverse(-1.0, kTimeoutMs);
    _rightMotor3.configPeakOutputForward(+1.0, kTimeoutMs);
		_rightMotor3.configPeakOutputReverse(-1.0, kTimeoutMs);

		/* FPID Gains for distance servo */
		_rightMotor1.config_kP(kSlot_Distance, kGains_Distance.kP, kTimeoutMs);
		_rightMotor1.config_kI(kSlot_Distance, kGains_Distance.kI, kTimeoutMs);
		_rightMotor1.config_kD(kSlot_Distance, kGains_Distance.kD, kTimeoutMs);
		_rightMotor1.config_kF(kSlot_Distance, kGains_Distance.kF, kTimeoutMs);
		_rightMotor1.config_IntegralZone(kSlot_Distance, kGains_Distance.kIzone, kTimeoutMs);
		_rightMotor1.configClosedLoopPeakOutput(kSlot_Distance, kGains_Distance.kPeakOutput, kTimeoutMs);

		/* FPID Gains for turn servo */
		_rightMotor1.config_kP(kSlot_Turning, kGains_Turning.kP, kTimeoutMs);
		_rightMotor1.config_kI(kSlot_Turning, kGains_Turning.kI, kTimeoutMs);
		_rightMotor1.config_kD(kSlot_Turning, kGains_Turning.kD, kTimeoutMs);
		_rightMotor1.config_kF(kSlot_Turning, kGains_Turning.kF, kTimeoutMs);
		_rightMotor1.config_IntegralZone(kSlot_Turning, kGains_Turning.kIzone, kTimeoutMs);
		_rightMotor1.configClosedLoopPeakOutput(kSlot_Turning, kGains_Turning.kPeakOutput, kTimeoutMs);
		
		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		_rightMotor1.configClosedLoopPeriod(0, closedLoopTimeMs, kTimeoutMs);
		_rightMotor1.configClosedLoopPeriod(1, closedLoopTimeMs, kTimeoutMs);

		/**
		 * configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		_rightMotor1.configAuxPIDPolarity(false, kTimeoutMs);

    zeroSensors();

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
    _leftMotor1.set(ControlMode.PercentOutput, speed);
  }

  /**
   * sets the speed of left motor 2
   * 
   * @param speed between -1 and 1 negative is reverse, positive is
   *        forward, 0 is stationary
   */
  private void driveleftMotor2(double speed) {
    _leftMotor2.set(ControlMode.PercentOutput, speed);
  }

  /**
   * sets the speed of left motor 3
   * 
   * @param speed between -1 and 1 negative is reverse, positive is
   *        forward, 0 is stationary
   */
  private void driveleftMotor3(double speed) {
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
    _rightMotor1.set(ControlMode.PercentOutput, speed);
  }

  /**
   * sets the speed of right motor 2
   * 
   * @param speed between -1 and 1 negative is reverse, positive is
   *        forward, 0 is stationary
   */
  private void driverightMotor2(double speed) {
    _rightMotor2.set(ControlMode.PercentOutput, speed);
  }

  /**
   * sets the speed of right motor 3
   * 
   * @param speed between -1 and 1 negative is reverse, positive is
   *        forward, 0 is stationary
   */
  private void driverightMotor3(double speed) {
    _rightMotor3.set(ControlMode.PercentOutput, speed);
  }
  
  /**
   * sets the speed of right motors 1, 2, and 3
   * 
   * @param speed 1.0 to -1.0. negative is reverse, positive is
   *              forward, 0 is stationary
   */
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
    return _leftMotor1.getSelectedSensorPosition(PID_PRIMARY)/(TICKS_PER_INCH);
  }

  public double getVelocity()
  {
    return _leftMotor1.getSelectedSensorVelocity(PID_PRIMARY)/(TICKS_PER_INCH_PER_100MS);
  }

  public double getHeading()
  {
    //return _pidgey.getFusedHeading();
    return _rightMotor1.getSelectedSensorPosition(PID_TURN)/PIGEON_UNITS_PER_DEGREE;
  }

  public void setSetPointPosition(double setPoint, double setAngle) {
    double target_sensorUnits = 2.0 * setPoint * TICKS_PER_INCH;
    double target_turn = setAngle * PIGEON_UNITS_PER_DEGREE;

    _rightMotor1.set(ControlMode.MotionMagic, target_sensorUnits, DemandType.AuxPID, target_turn);
    _rightMotor2.follow(_rightMotor1, FollowerType.PercentOutput);
    _rightMotor3.follow(_rightMotor1, FollowerType.PercentOutput);
    _leftMotor1.follow(_rightMotor1, FollowerType.AuxOutput1);
    _leftMotor2.follow(_rightMotor1, FollowerType.AuxOutput1);
    _leftMotor3.follow(_rightMotor1, FollowerType.AuxOutput1);
  }

  public double getErrorPosition() {
    return (_rightMotor1.getActiveTrajectoryPosition(PID_PRIMARY)-_rightMotor1.getSelectedSensorPosition(PID_PRIMARY))/(TICKS_PER_INCH);
    //return _leftMotor1.getClosedLoopError(kPIDLoopIdx)/TICKS_PER_REV;--only for nonMotionMagic or nonMotion Profile
  }

  public double getErrorHeading() {
    return (_rightMotor1.getActiveTrajectoryPosition(PID_TURN)-_rightMotor1.getSelectedSensorPosition(PID_TURN))/PIGEON_UNITS_PER_DEGREE; 
  }

  /**
   * Zero all sensors, drivetrain encoders and Pigeon IMU
   */
  public void zeroSensors() {
    zeroDistance();
    zeroIMU();
    // System.out.println("[Quadrature Encoders + Pigeon] All sensors are zeroed.\n");
  }

  /** Zero QuadEncoders, used to reset position when initializing Motion Magic */
  public void zeroDistance(){
    _leftMotor1.getSensorCollection().setIntegratedSensorPosition(0, kTimeoutMs);
    _rightMotor1.getSensorCollection().setIntegratedSensorPosition(0, kTimeoutMs);
    //System.out.println("[Integrated Encoders] All encoders are zeroed.\n");
  }
  
  public void zeroIMU() {
    _pidgey.setYaw(0, kTimeoutMs);
    _pidgey.setAccumZAngle(0, kTimeoutMs);
    _pidgey.setFusedHeading(0, kTimeoutMs);
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoystick());
  }

}