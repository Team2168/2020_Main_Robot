/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import org.team2168.Constants;
import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */

  /* Singleton Constructor */
  public static Drivetrain instance = null;

  /* Motor controllers */
  private static WPI_TalonFX _leftMotor1;
  private static WPI_TalonFX _leftMotor2;
  private static WPI_TalonFX _leftMotor3;
  private static WPI_TalonFX _rightMotor1;
  private static WPI_TalonFX _rightMotor2;
  private static WPI_TalonFX _rightMotor3;

  /* Left/Right drivetrain split into speed contollers */
  private static SpeedControllerGroup _leftMotors;
  private static SpeedControllerGroup _rightMotors;

  /* IMU */
  private static PigeonIMU _pidgey;

  /* Config Objects for motor controllers */
  TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
  TalonFXConfiguration _rightConfig = new TalonFXConfiguration();

  /* Odometry class for tracking pose */
  private static DifferentialDriveOdometry _odometry;
  

  /* Local constants and config */
  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 40; // amps
  private final double TRIGGER_THRESHOLD_LIMIT = 60; // amp
  private final double TRIGGER_THRESHOLD_TIME = 0.2; // s
  public static final boolean DT_REVERSE_LEFT1 = false;
  public static final boolean DT_REVERSE_LEFT2 = false;
  public static final boolean DT_REVERSE_LEFT3 = false;
  public static final boolean DT_REVERSE_RIGHT1 = true;
  public static final boolean DT_REVERSE_RIGHT2 = true;
  public static final boolean DT_REVERSE_RIGHT3 = true;
  public static final boolean DT_3_MOTORS_PER_SIDE = true;

  private static final double TICKS_PER_REV = 2048.0; // one event per edge on each quadrature channel
  private static final double TICKS_PER_100MS = TICKS_PER_REV / 10.0;
  private static final double GEAR_RATIO = (50.0 / 10.0) * (40.0 / 22.0);
  private static final double WHEEL_DIAMETER = 6.0; // inches
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // inches
  private static final double PIGEON_UNITS_PER_ROTATION = 8192.0;;
  private static final double DEGREES_PER_REV = 360.0;
  private static final double PIGEON_UNITS_PER_DEGREE = PIGEON_UNITS_PER_ROTATION / 360;
  private static final double WHEEL_BASE = 24.0; // distance between wheels (width) in inches

  /** Invert Directions for Left and Right */
  // TalonFXInvertType _leftInvert = TalonFXInvertType.CounterClockwise; //Same as
  // invert = "false"
  // TalonFXInvertType _rightInvert = TalonFXInvertType.Clockwise; //Same as
  // invert = "true"
  Boolean _leftInvert = false;
  Boolean _rightInvert = true;




  public Drivetrain() {
    System.out.println("CAN Comp Bot Drivetrain enabled - 6 motors");
    _leftMotor1 = new WPI_TalonFX(RobotMap.DRIVETRAIN_LEFT_MOTOR_1_PDP);
    _leftMotor2 = new WPI_TalonFX(RobotMap.DRIVETRAIN_LEFT_MOTOR_2_PDP);
    _leftMotor3 = new WPI_TalonFX(RobotMap.DRIVETRAIN_LEFT_MOTOR_3_PDP);
    _rightMotor1 = new WPI_TalonFX(RobotMap.DRIVETRAIN_RIGHT_MOTOR_1_PDP);
    _rightMotor2 = new WPI_TalonFX(RobotMap.DRIVETRAIN_RIGHT_MOTOR_2_PDP);
    _rightMotor3 = new WPI_TalonFX(RobotMap.DRIVETRAIN_RIGHT_MOTOR_3_PDP);
    _pidgey = new PigeonIMU(17);

    /* Component configuration */
    _leftMotor1.configFactoryDefault();
    _leftMotor2.configFactoryDefault();
    _leftMotor3.configFactoryDefault();
    _rightMotor1.configFactoryDefault();
    _rightMotor2.configFactoryDefault();
    _rightMotor3.configFactoryDefault();
    _pidgey.configFactoryDefault();



    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CONTINUOUS_CURRENT_LIMIT,
        TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    _leftMotor1.configSupplyCurrentLimit(talonCurrentLimit);
    _leftMotor2.configSupplyCurrentLimit(talonCurrentLimit);
    _leftMotor3.configSupplyCurrentLimit(talonCurrentLimit);
    _rightMotor1.configSupplyCurrentLimit(talonCurrentLimit);
    _rightMotor2.configSupplyCurrentLimit(talonCurrentLimit);
    _rightMotor3.configSupplyCurrentLimit(talonCurrentLimit);

    _leftConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    _rightConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    _leftConfig.neutralDeadband = Constants.kNeutralDeadband;
    _rightConfig.neutralDeadband = Constants.kNeutralDeadband;

    _leftMotor1.configAllSettings(_leftConfig);
    _rightMotor1.configAllSettings(_rightConfig);

    // _leftMotor1.setInverted(_leftInvert);
    // _leftMotor2.setInverted(_leftInvert);
    // _leftMotor3.setInverted(_leftInvert);
    // _rightMotor1.setInverted(_rightInvert);
    // _rightMotor2.setInverted(_rightInvert);
    // _rightMotor3.setInverted(_rightInvert);

    /* Odometry for pose tracking */
    _odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    /* Component abstraction */
    _leftMotors = new SpeedControllerGroup(_leftMotor1, _leftMotor2, _leftMotor3);
    _rightMotors = new SpeedControllerGroup(_rightMotor1, _rightMotor2, _rightMotor3);

    /* Config for abstracted components */
    // Motor controller level inversion doesn't work for some reason
    _leftMotors.setInverted(_leftInvert);
    _rightMotors.setInverted(_rightInvert);

    resetEncoders();
    zeroHeading();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _odometry.update(Rotation2d.fromDegrees(getHeading()), ticksToMeters(_leftMotor1.getSelectedSensorPosition()),
        ticksToMeters(_rightMotor1.getSelectedSensorPosition()));
  }

  /**
   * Sets speed of the left Motors
   * @param leftSpeed Double between -1.0 and 1.0
   */
  public void setLeftMotors(double leftSpeed) {
    _leftMotors.set(leftSpeed);
  }

  /**
   * Sets speed of the right motors
   * @param rightSpeed Double between -1.0 and 1.0
   */
  public void setRightMotors(double rightSpeed) {
    _rightMotors.set(rightSpeed);
  }

  /**
   * (From {@link edu.wpi.first.wpilibj.SpeedController})
   * Sets the voltage output of the SpeedController.  Compensates for the current bus
   * voltage to ensure that the desired voltage is output even if the battery voltage is below
   * 12V - highly useful when the voltage outputs are "meaningful" (e.g. they come from a
   * feedforward calculation).
   *
   * <p>NOTE: This function *must* be called regularly in order for voltage compensation to work
   * properly - unlike the ordinary set function, it is not "set it and forget it."
   * 
   * @param volts voltage to set motors
   * @param batteryVoltage voltage of the battery.
   */
  public void setLeftMotorsVolts(double volts, double batteryVoltage) {
    _leftMotors.set(volts/batteryVoltage);
  }

    /**
   * (From {@link edu.wpi.first.wpilibj.SpeedController})
   * Sets the voltage output of the SpeedController.  Compensates for the current bus
   * voltage to ensure that the desired voltage is output even if the battery voltage is below
   * 12V - highly useful when the voltage outputs are "meaningful" (e.g. they come from a
   * feedforward calculation).
   *
   * <p>NOTE: This function *must* be called regularly in order for voltage compensation to work
   * properly - unlike the ordinary set function, it is not "set it and forget it."
   * 
   * @param volts voltage to set motors
   * @param batteryVoltage voltage of the battery.
   */
  public void setRightMotorsVolts(double volts, double batteryVoltage) {
    _rightMotors.set(volts/batteryVoltage);
  }

  /**
   * sets drivetrain speed
   * @param leftSpeed Double between -1.0 and 1.0
   * @param rightSpeed Double between -1.0 and 1.0
   */
  public void set(double leftSpeed, double rightSpeed) {
    setLeftMotors(leftSpeed);
    setRightMotors(rightSpeed);
  }

  /**
   * (From {@link edu.wpi.first.wpilibj.SpeedController})
   * Sets the voltage output of the SpeedController.  Compensates for the current bus
   * voltage to ensure that the desired voltage is output even if the battery voltage is below
   * 12V - highly useful when the voltage outputs are "meaningful" (e.g. they come from a
   * feedforward calculation).
   *
   * <p>NOTE: This function *must* be called regularly in order for voltage compensation to work
   * properly - unlike the ordinary set function, it is not "set it and forget it."
   * 
   * @param leftVolts left voltage
   * @param rightVolts right voltage
   */
  // public void setVolts(double leftVolts, double rightVolts) {
  //   setLeftMotorsVolts(leftVolts);
  //   setRightMotorsVolts(rightVolts);
  //   SmartDashboard.putNumber("left volts", leftVolts);
  //   SmartDashboard.putNumber("right volts", rightVolts);
  // }
  public void setVolts(double leftVolts, double rightVolts) {
    double voltage = RobotController.getBatteryVoltage();
    setLeftMotorsVolts(leftVolts, voltage);
    setRightMotorsVolts(rightVolts, voltage);
    SmartDashboard.putNumber("left volts", leftVolts);
    SmartDashboard.putNumber("right volts", rightVolts);
  }

  /**
   * Gets the current x, y position and rotation of the robot on the field
   * This is mostly used for Ramsete command autos
   * @return Pose2d the current pose of the robot
   */
  public Pose2d getPose() {
    return _odometry.getPoseMeters();
  }

  /**
   * returns rate in ticks
   * This is mainly used for ramsetecommand autos
   * 
   * @return DifferentialDriveWheelSpeeds wheel speeds
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(ticksToMeters(_leftMotor1.getSelectedSensorVelocity()),
        ticksToMeters(_rightMotor1.getSelectedSensorVelocity()));
  }

  /**
   * Resets drivetrain odometry
   * @param pose Pose2d of the robot
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    _odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Zeroes the drivetrain encoders
   */
  public void resetEncoders() {
    _leftMotor1.setSelectedSensorPosition(0);
    _rightMotor1.setSelectedSensorPosition(0);
  }

  /**
   * gets the average encoder position
   * 
   * @return average encoder distance in ticks
   */
  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  /**
   * Gets left encoder position
   * @return encoder distance in ticks
   */
  public double getLeftEncoderDistance() {
    return (_leftInvert ? -_leftMotor1.getSelectedSensorPosition() : _leftMotor1.getSelectedSensorPosition());
  }

  /**
   * Gets left encoder position
   * @return encoder distance in ticks
   */
  public double getRightEncoderDistance() {
    return (_rightInvert ? -_rightMotor1.getSelectedSensorPosition() : _rightMotor1.getSelectedSensorPosition());
  }

  /**
   * Zeroes imu heading
   */
  public void zeroHeading() {
    _pidgey.setFusedHeading(0);
  }

  /**
   * Gets IMU heading
   * @return IMU heading in degrees
   */
  public double getHeading() {
    return _pidgey.getFusedHeading();
  }

  /**
   * Gets motor voltages
   * Format is {leftMotor1, leftMotor2, leftMotor3, rightMotor1, rightMotor2, rightMotor3}
   * @return array containing motor bus voltages
   */
  public Double[] getVoltages() {
    return new Double[] { _leftMotor1.getBusVoltage(), _leftMotor2.getBusVoltage(), _leftMotor3.getBusVoltage(),
        _rightMotor1.getBusVoltage(), _rightMotor2.getBusVoltage(), _rightMotor3.getBusVoltage() };
  }

  /**
   * Gets drivetrain singleton
   * @return the instance of Drivetrain
   */
  public static Drivetrain getInstance() {
    if (instance == null)
      instance = new Drivetrain();
    return instance;
  }

  public void setAllMotorsBrake() {
    _leftMotor1.setNeutralMode(NeutralMode.Brake);
    _leftMotor2.setNeutralMode(NeutralMode.Brake);
    _leftMotor3.setNeutralMode(NeutralMode.Brake);
    _rightMotor1.setNeutralMode(NeutralMode.Brake);
    _rightMotor2.setNeutralMode(NeutralMode.Brake);
    _rightMotor3.setNeutralMode(NeutralMode.Brake);
  }

  public void setAllMotorsCoast() {
    _leftMotor1.setNeutralMode(NeutralMode.Coast);
    _leftMotor2.setNeutralMode(NeutralMode.Coast);
    _leftMotor3.setNeutralMode(NeutralMode.Coast);
    _rightMotor1.setNeutralMode(NeutralMode.Coast);
    _rightMotor2.setNeutralMode(NeutralMode.Coast);
    _rightMotor3.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Converts a setpoint in degrees to IMU 'encoder ticks'
   * 
   * @param setpoint
   * @return
   */
  private double degrees_to_ticks(double setpoint) {
    // return (setpoint / DEGREES_PER_REV) * PIGEON_UNITS_PER_ROTATION / 2.0;
    return setpoint * 10.0;
  }

  /**
   * Converts a setpoint in IMU 'encoder ticks', to degrees
   * 
   * @param setpoint
   * @return
   */
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

  private double ticksToMeters(double setpoint) {
    // TODO is this the best way of accomplishing this?
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

  private double degrees_to_wheel_revs(double degrees) {
    return (degrees / 360.0) * (WHEEL_BASE / WHEEL_DIAMETER);
  }
}
