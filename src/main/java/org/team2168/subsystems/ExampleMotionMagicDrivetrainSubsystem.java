/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import org.team2168.Gains;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 * Add your docs here.
 */
public class ExampleMotionMagicDrivetrainSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  /** ---- Flat constants, you should not need to change these ---- */
  /* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
  public final static int REMOTE_0 = 0;
  public final static int REMOTE_1 = 1;
  /* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
  public final static int PID_PRIMARY = 0;
  public final static int PID_TURN = 1;
  /* Firmware currently supports slots [0, 3] and can be used for either PID Set */
  public final static int SLOT_0 = 0;
  public final static int SLOT_1 = 1;
  public final static int SLOT_2 = 2;
  public final static int SLOT_3 = 3;
  /* ---- Named slots, used to clarify code ---- */
  public final static int kSlot_Distance = SLOT_0;
  public final static int kSlot_Turning = SLOT_1;
  public final static int kSlot_Velocity = SLOT_2;
  public final static int kSlot_MotProf = SLOT_3; 

  /**
   * set to zero to skip waiting for confirmation, set to nonzero to wait and
   * report to DS if action fails.
   */
  public static final int kTimeoutMs = 30;

    /**
   * Motor neutral dead-band, set to the minimum 0.1%.
   */
  public final static double kNeutralDeadband = 0.001;

  /**
   * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
     * Not all set of Gains are used in this project and may be removed as desired.
     * 
   * 	                                    			  kP   kI   kD   kF               Iz    PeakOut */
  public final static Gains kGains_Distance = new Gains( 0.1, 0.0,  0.0, 0.0,            100,  0.50 );
  public final static Gains kGains_Turning = new Gains( 2.0, 0.0,  4.0, 0.0,            200,  1.00 );
  public final static Gains kGains_Velocity = new Gains( 0.1, 0.0, 20.0, 1023.0/6800.0,  300,  0.50 );
  public final static Gains kGains_MotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/6800.0,  400,  1.00 );
  /**
   * Convert target RPM to ticks / 100ms.
   * 256*4x (quadrature encoder) Ticks/Rev *  RPM / 600 100ms/min in either direction:
   * velocity setpoint is in units/100ms
   */
  final double TICKS_PER_REV = 256.0 * 4.0; //one event per edge on each quadrature channel
  final double TICKS_PER_100MS = TICKS_PER_REV / 600.0;
  final double NUM_REVOLUTIONS = 32.0;

  /**
   * This is a property of the Pigeon IMU, and should not be changed.
   */
  public final static int kPigeonUnitsPerRotation = 8192;
  public final static double PIGEON_UNITS_PER_DEGREE = kPigeonUnitsPerRotation; //TODO figure out with James

  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 30; //amps
  private final double TRIGGER_THRESHOLD_LIMIT = 60; //amp
  private final double TRIGGER_THRESHOLD_TIME = 500; //ms

  public TalonSRX _talon;
  public TalonSRX _talonFollow;
  PigeonIMU _pidgey;


  private static ExampleMotionMagicDrivetrainSubsystem _instance;


  private ExampleMotionMagicDrivetrainSubsystem()
  {
    _talon = new TalonSRX(11);
    _talonFollow = new TalonSRX(10);
    _pidgey = new PigeonIMU(3);
    
    /* Factory Default all hardware to prevent unexpected behaviour */
    _talon.configFactoryDefault();
    _talonFollow.configFactoryDefault();
    _pidgey.configFactoryDefault();

        /* Set Neutral Mode */
    _talonFollow.setNeutralMode(NeutralMode.Brake);
    _talon.setNeutralMode(NeutralMode.Brake);
    /** Feedback Sensor Configuration */
    
    /* Configure the left Talon's selected sensor as local QuadEncoder */
    _talonFollow.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,				// Local Feedback Source
                                                PID_PRIMARY,					// PID Slot for Source [0, 1]
                                                kTimeoutMs);					// Configuration Timeout

    /* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
    _talon.configRemoteFeedbackFilter(_talonFollow.getDeviceID(),					// Device ID of Source
                                        RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
                                        REMOTE_0,							// Source number [0, 1]
                                        kTimeoutMs);						// Configuration Timeout

    /* Configure the Pigeon IMU to the other remote slot available on the right Talon */
    _talon.configRemoteFeedbackFilter(_pidgey.getDeviceID(),
                                        RemoteSensorSource.Pigeon_Yaw,
                                        REMOTE_1,	
                                        kTimeoutMs);
    /* Setup Sum signal to be used for Distance */
    _talon.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, kTimeoutMs);				// Feedback Device of Remote Talon
    _talon.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, kTimeoutMs);	// Quadrature Encoder of current Talon
    
    /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
    _talon.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
                          PID_PRIMARY,
                          kTimeoutMs);
    
    /* Scale Feedback by 0.5 to half the sum of Distance */
    _talon.configSelectedFeedbackCoefficient(	0.5, 						// Coefficient
                            PID_PRIMARY,		// PID Slot of Source 
                            kTimeoutMs);		// Configuration Timeout
    
    /* Configure Remote 1 [Pigeon IMU's Yaw] to be used for Auxiliary PID Index */
    _talon.configSelectedFeedbackSensor(	FeedbackDevice.RemoteSensor1,
                          PID_TURN,
                          kTimeoutMs);


        /**
     * Phase sensor accordingly. 
         * Positive Sensor Reading should match Green (blinking) Leds on Talon
         */
    _talon.setSensorPhase(true);
    _talonFollow.setSensorPhase(true);

    /***
     * invert motors if necessary 
     */
    _talon.setInverted(true); //TO-DO
    _talonFollow.setInverted(true);

    /* Set status frame periods to ensure we don't have stale data */
    _talon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, kTimeoutMs);
    _talon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, kTimeoutMs);
    _talon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, kTimeoutMs);
    _talon.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, kTimeoutMs);
    _talonFollow.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);
    _pidgey.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, kTimeoutMs);
    
        /* Configure neutral deadband */
    _talon.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);
    _talonFollow.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);

    /* Set the peak and nominal outputs */
    _talon.configNominalOutputForward(0, kTimeoutMs);
    _talon.configNominalOutputReverse(0, kTimeoutMs);
    _talon.configPeakOutputForward(1, kTimeoutMs);
    _talon.configPeakOutputReverse(-1, kTimeoutMs);
    _talonFollow.configNominalOutputForward(0, kTimeoutMs);
    _talonFollow.configNominalOutputReverse(0, kTimeoutMs);
    _talonFollow.configPeakOutputForward(1, kTimeoutMs);
    _talonFollow.configPeakOutputReverse(-1, kTimeoutMs);


    /* FPID Gains for distance servo */
    _talon.config_kP(kSlot_Distance, kGains_Distance.kP, kTimeoutMs);
    _talon.config_kI(kSlot_Distance, kGains_Distance.kI, kTimeoutMs);
    _talon.config_kD(kSlot_Distance, kGains_Distance.kD, kTimeoutMs);
    _talon.config_kF(kSlot_Distance, kGains_Distance.kF, kTimeoutMs);
    _talon.config_IntegralZone(kSlot_Distance, kGains_Distance.kIzone, kTimeoutMs);
    _talon.configClosedLoopPeakOutput(kSlot_Distance, kGains_Distance.kPeakOutput, kTimeoutMs);

    /* FPID Gains for turn servo */
    _talon.config_kP(kSlot_Turning, kGains_Turning.kP, kTimeoutMs);
    _talon.config_kI(kSlot_Turning, kGains_Turning.kI, kTimeoutMs);
    _talon.config_kD(kSlot_Turning, kGains_Turning.kD, kTimeoutMs);
    _talon.config_kF(kSlot_Turning, kGains_Turning.kF, kTimeoutMs);
    _talon.config_IntegralZone(kSlot_Turning, kGains_Turning.kIzone, kTimeoutMs);
    _talon.configClosedLoopPeakOutput(kSlot_Turning, kGains_Turning.kPeakOutput, kTimeoutMs);
    
        /**
     * 1ms per loop.  PID loop can be slowed down if need be.
     * For example,
     * - if sensor updates are too slow
     * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
     * - sensor movement is very slow causing the derivative error to be near zero.
     */
    int closedLoopTimeMs = 1;
    _talon.configClosedLoopPeriod(0, closedLoopTimeMs, kTimeoutMs);
    _talon.configClosedLoopPeriod(1, closedLoopTimeMs, kTimeoutMs);

    /**
     * configAuxPIDPolarity(boolean invert, int timeoutMs)
     * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
     * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
     */
    _talon.configAuxPIDPolarity(false, kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    _talon.configMotionCruiseVelocity((int) (1600.0*TICKS_PER_100MS), kTimeoutMs);
    _talon.configMotionAcceleration((int) (800.0*TICKS_PER_100MS), kTimeoutMs);

    zeroSensors();

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
    CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    _talon.configSupplyCurrentLimit(talonCurrentLimit);
    _talonFollow.configSupplyCurrentLimit(talonCurrentLimit);

    //set second motor as a follow
    _talonFollow.follow(_talon, FollowerType.PercentOutput);

    _talon.selectProfileSlot(kSlot_Distance, PID_PRIMARY);
    _talon.selectProfileSlot(kSlot_Turning, PID_TURN);

    ConsolePrinter.putNumber("Motion Magic Position", () -> {return getPosition();}, true, false);
    ConsolePrinter.putNumber("Motion Magic Velocity", () -> {return getVelocity();}, true, false);
    ConsolePrinter.putNumber("Motion Magic Error", () -> {return getErrorPosition();}, true, false);
    ConsolePrinter.putNumber("Motor Output Percent", () -> {return _talon.getMotorOutputPercent();}, true, false);
    ConsolePrinter.putNumber("Setpoint Position", () -> {return _talon.getActiveTrajectoryPosition()/TICKS_PER_REV;}, true, false);
    // ConsolePrinter.putNumber("Setpoint Velocity", () -> {return _talon.getActiveTrajectoryVelocity()/TICKS_PER_100MS;}, true, false);
    // ConsolePrinter.putNumber("Setpoint Heading", () -> {return _talon.getActiveTrajectoryPosition(1)/TICKS_PER_REV;}, true, false); //need to convert from gyro sensor units
    /**
     * we think that in MotionMagicMode:
     * closedLoopError is the error to where we should be instantaneously
     * closedLoopTarget is the instantaneous position setpoint (not the end position setpoint)
     */




  }
  /**
   * singleton constructor of the ExampleMotionMagicSubsystem 
   * @return
   */
  public static ExampleMotionMagicDrivetrainSubsystem getInstance()
  {
    if (_instance == null)
      _instance = new ExampleMotionMagicDrivetrainSubsystem();
    return _instance;
  }

  public void drive(double speed)
  {
    _talon.set(ControlMode.PercentOutput, speed);
  }

  public double getPosition()
  {
    return _talon.getSelectedSensorPosition(PID_PRIMARY)/TICKS_PER_REV;
  }

  public double getVelocity()
  {
    return _talon.getSelectedSensorVelocity(PID_PRIMARY)/TICKS_PER_100MS;
  }

  public double getHeading()
  {
    return _talon.getSelectedSensorPosition(PID_TURN);
  }

  public void setSetPoint(double setPoint, double setAngle)
  {
    _talon.set(ControlMode.MotionMagic, setPoint*TICKS_PER_REV, DemandType.AuxPID, setAngle*PIGEON_UNITS_PER_DEGREE);
  }

  public double getErrorPosition()
  {
    return (_talon.getActiveTrajectoryPosition()-_talon.getSelectedSensorPosition(PID_PRIMARY))/TICKS_PER_REV;
    //return _talon.getClosedLoopError(kPIDLoopIdx)/TICKS_PER_REV;--only for nonMotionMagic or nonMotion Profile
  }

  public double getErrorHeading()
  {
    return (_talon.getActiveTrajectoryPosition(PID_PRIMARY)-_talon.getSelectedSensorPosition(PID_PRIMARY))/PIGEON_UNITS_PER_DEGREE;
  }

  /** Zero all sensors, both Talons and Pigeon */
  public void zeroSensors() {
    _talonFollow.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
    _talon.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
    _pidgey.setYaw(0, kTimeoutMs);
    _pidgey.setAccumZAngle(0, kTimeoutMs);
   // System.out.println("[Quadrature Encoders + Pigeon] All sensors are zeroed.\n");
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}