/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import org.team2168.Constants;
import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;


public class DrivetrainNew extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // there may be only one
  public static DrivetrainNew instance = null;

  private static WPI_TalonFX  _leftMotor1;
  private static WPI_TalonFX  _leftMotor2;
  private static WPI_TalonFX  _leftMotor3;
  private static WPI_TalonFX _rightMotor1;
  private static WPI_TalonFX _rightMotor2;
  private static WPI_TalonFX _rightMotor3;

  

  private static SpeedControllerGroup _leftMotors;
  private static SpeedControllerGroup _rightMotors;

  // auto control stuff
  private static RamseteController ramseteController;
  private static SimpleMotorFeedforward driveFeedforward;
  private static DifferentialDriveKinematics driveKinematics;
  private static PIDController leftDriveController;
  private static PIDController rightDriveController;

  private static PigeonIMU _pidgey;

  private static DifferentialDrive _drive;

  // stuff I yoinked from the old drivetrain (i dunno if it's needed; TODO ?)
  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 40; //amps
  private final double TRIGGER_THRESHOLD_LIMIT = 60; //amp
  private final double TRIGGER_THRESHOLD_TIME = 0.2; //s
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
  // end of yoinking

  //odometry class for tracking pose
  private static DifferentialDriveOdometry _odometry;

  private DrivetrainNew() {
    System.out.println("CAN Comp Bot Drivetrain enabled - 6 motors");
    _leftMotor1 = new WPI_TalonFX(RobotMap.DRIVETRAIN_LEFT_MOTOR_1_PDP);
    _leftMotor2 = new WPI_TalonFX(RobotMap.DRIVETRAIN_LEFT_MOTOR_2_PDP);
    _leftMotor3 = new WPI_TalonFX(RobotMap.DRIVETRAIN_LEFT_MOTOR_3_PDP);
    _rightMotor1 = new WPI_TalonFX(RobotMap.DRIVETRAIN_RIGHT_MOTOR_1_PDP);
    _rightMotor2 = new WPI_TalonFX(RobotMap.DRIVETRAIN_RIGHT_MOTOR_2_PDP);
    _rightMotor3 = new WPI_TalonFX(RobotMap.DRIVETRAIN_RIGHT_MOTOR_3_PDP);
    _pidgey = new PigeonIMU(17);

    _leftMotor1.configFactoryDefault();
    _leftMotor2.configFactoryDefault();
    _leftMotor3.configFactoryDefault();
    _rightMotor1.configFactoryDefault();
    _rightMotor2.configFactoryDefault();
    _rightMotor3.configFactoryDefault();
    _pidgey.configFactoryDefault();

    // current limit stuff
    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
    CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    _leftMotor1.configSupplyCurrentLimit(talonCurrentLimit);
    _leftMotor2.configSupplyCurrentLimit(talonCurrentLimit);
    _leftMotor3.configSupplyCurrentLimit(talonCurrentLimit);
    _rightMotor1.configSupplyCurrentLimit(talonCurrentLimit);
    _rightMotor2.configSupplyCurrentLimit(talonCurrentLimit);
    _rightMotor3.configSupplyCurrentLimit(talonCurrentLimit);

    // inversion stuff
    _leftMotor1.setInverted(_leftInvert);
    _leftMotor2.setInverted(_leftInvert);
    _leftMotor3.setInverted(_leftInvert);
    _rightMotor1.setInverted(_rightInvert);
    _rightMotor2.setInverted(_rightInvert);
    _rightMotor3.setInverted(_rightInvert);

    // encoder stuff
    _leftConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    _rightConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    _leftConfig.neutralDeadband = Constants.kNeutralDeadband;
    _rightConfig.neutralDeadband = Constants.kNeutralDeadband;

    _leftMotor1.configAllSettings(_leftConfig);
    _rightMotor1.configAllSettings(_rightConfig);

    



    _leftMotors = new SpeedControllerGroup(_leftMotor1, _leftMotor2, _leftMotor3);
    _rightMotors = new SpeedControllerGroup(_rightMotor1, _rightMotor2, _rightMotor3);
    _drive = new DifferentialDrive(_leftMotors, _rightMotors);
    _odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    // auto
    ramseteController = new RamseteController();
    driveFeedforward = new SimpleMotorFeedforward(Constants.kDriveS, Constants.kDriveV, Constants.kDriveA);
    driveKinematics = new DifferentialDriveKinematics(Constants.kTrackWidthMeters);
    leftDriveController = new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD);
    rightDriveController = new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD);


    resetEncoders();
  }

  
  @Override
  public void periodic() {
    //update the odometry
    _odometry.update(Rotation2d.fromDegrees(getHeading()), ticks_to_meters(_leftMotor1.getSelectedSensorPosition()), ticks_to_meters(_rightMotor1.getSelectedSensorPosition()));
  }

  private void setLeftMotor1(double leftSpeed) {
    _leftMotor1.set(ControlMode.PercentOutput, leftSpeed);
  }
  private void setLeftMotor2(double leftSpeed) {
    _leftMotor2.set(ControlMode.PercentOutput, leftSpeed);
  }
  private void setLeftMotor3(double leftSpeed) {
    _leftMotor3.set(ControlMode.PercentOutput, leftSpeed);
  }

  public void setLeftMotors(double leftSpeed) {
    setLeftMotor1(leftSpeed);
    setLeftMotor2(leftSpeed);
    setLeftMotor3(leftSpeed);
  }


  private void setRightMotor1(double rightSpeed) {
    _rightMotor1.set(ControlMode.PercentOutput, rightSpeed);
  }
  private void setRightMotor2(double rightSpeed) {
    _rightMotor2.set(ControlMode.PercentOutput, rightSpeed);
  }
  private void setRightMotor3(double rightSpeed) {
    _rightMotor3.set(ControlMode.PercentOutput, rightSpeed);
  }

  public void setRightMotors(double rightSpeed) {
    setRightMotor1(rightSpeed);
    setRightMotor2(rightSpeed);
    setRightMotor3(rightSpeed);
  }

  public void set(double leftSpeed, double rightSpeed) {
    setLeftMotors(leftSpeed);
    setRightMotors(rightSpeed);
  }
  
  public void setVoltage(double leftVolts, double rightVolts) {
    _leftMotors.setVoltage(leftVolts);
    _rightMotors.setVoltage(-rightVolts);
  }

  public Pose2d getPose() {
    return _odometry.getPoseMeters();
  }
  /**
   * returns rate in ticks
   * @return
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(_leftMotor1.getSelectedSensorVelocity(), _rightMotor1.getSelectedSensorVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    _odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void arcadeDrive(double fwd, double rot) {
    _drive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    _leftMotors.setVoltage(leftVolts);
    _rightMotors.setVoltage(-rightVolts);
    _drive.feed();
  }

  public void resetEncoders() {
    _leftMotor1.setSelectedSensorPosition(0);
    _rightMotor1.setSelectedSensorPosition(0);
  }

  /**
   * gets the average encoder position
   * @return average encoder distance in ticks
   */
  public double getAverageEncoderDistance() {
    return (_leftMotor1.getSelectedSensorPosition() + _rightMotor1.getSelectedSensorPosition())/2.0;
  }

  /**
   * sets the max output, used for scaling to drive more slowly
   * just in here because the docs had it :/
   * @param maxOutput  Multiplied with the output percentage computed by the drive functions.
   */
  public void setMaxOutput(double maxOutput) {
    _drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    _pidgey.setYaw(0);
  }

  // public double getRawYaw() {
  //   double[] ypr = {0.0, 0.0, 0.0};
  //   _pidgey.getYawPitchRoll(ypr);
  //   return ypr[0];
    
  // }

  public double getHeading() {
    // return Math.abs(Math.IEEEremainder(getRawYaw()%360, 360)); //TODO seems a bit wierd idk
    return _pidgey.getFusedHeading();
  }


  // TODO do I need a get turn rate? 

  private int getLeftPosition() {
    return _leftMotor1.getSelectedSensorPosition();
  }

  private int getRightPosition() {
    return _rightMotor1.getSelectedSensorPosition();
  }

  public RamseteController getRamseteController() {
    return ramseteController;
  }

  public SimpleMotorFeedforward getFeedForward() {
    return driveFeedforward;
  }

  public DifferentialDriveKinematics getDriveKinematics() {
    return driveKinematics;
  }

  public PIDController getLeftDriveController() {
    return leftDriveController;
  }

  public PIDController getRightDriveController() {
    return rightDriveController;
  }

  
  
  // TODO add getTurnRate
  // public double getTurnRate() {
  //   return _pidgey.get
  // }


  public static DrivetrainNew getInstance() {
    if (instance == null)
      instance = new DrivetrainNew();
    return instance;
  }

   /**
   * Converts a setpoint in degrees to IMU 'encoder ticks'
   * @param setpoint
   * @return
   */
  private double degrees_to_ticks(double setpoint) {
    // return (setpoint / DEGREES_PER_REV) * PIGEON_UNITS_PER_ROTATION / 2.0;
    return setpoint * 10.0;
  }

  private double ticks_to_degrees(double setpoint) {
    // return (setpoint / PIGEON_UNITS_PER_ROTATION) * DEGREES_PER_REV * 2.0; 
    return setpoint / 10.0;
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

  private double ticks_to_meters(double setpoint) {
    // TODO the best way of accomplishing this?
    return (ticks_to_inches(setpoint) / 39.37008);
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
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
