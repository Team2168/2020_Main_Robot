package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import org.team2168.Gains;
import org.team2168.RobotMap;
import org.team2168.commands.shooter.DriveShooterWithJoystick;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Shooter extends Subsystem {

    private TalonFX _motorOne;
    private TalonFX _motorTwo;

    // private final boolean _motorOneReversed = false;
    // private final boolean _motorTwoReversed = false;

    private StatorCurrentLimitConfiguration talonCurrentLimit;
    private final boolean ENABLE_CURRENT_LIMIT = true;
    private final double CONTINUOUS_CURRENT_LIMIT = 30; //amps
    private final double TRIGGER_THRESHOLD_LIMIT = 35; //amp
    private final double TRIGGER_THRESHOLD_TIME = 100; //ms

    private static Shooter _instance;

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

    public static final double max_velocity = 8000.0; //TODO set (measured ~18,000 units/1000ms at full stick)

    /** Invert Directions for Left and Right */
    TalonFXInvertType _motorOneInvert = TalonFXInvertType.Clockwise;
    TalonFXInvertType _motorTwoInvert = TalonFXInvertType.CounterClockwise;

    /** Config Objects for motor controllers */
    TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration _rightConfig = new TalonFXConfiguration();

    private static final double TICKS_PER_REV = 2048.0; //one event per edge on each quadrature channel
    private static final double TICKS_PER_100MS = TICKS_PER_REV / 10.0;
    private static final double GEAR_RATIO = 24.0/18.0;  // motor pulley/shooter wheel pulley
    private static final double SECS_PER_MIN = 60.0;

    /**
     * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 7200 represents Velocity units at 100% output
     * 
     * 	                                      kP    kI    kD          kF               Iz   PeakOut
     */
    final Gains kGains_Velocity = new Gains( 0.775, 0.000, 0, 0.17825/TICKS_PER_100MS,  300,  1.00); // kF = 1023*0.00016/ticks_per_100ms
    
    private double setPointVelocity_sensorUnits;

    private Shooter() {
        _motorOne = new TalonFX(RobotMap.SHOOTER_MOTOR_ONE_PDP);
        _motorTwo = new TalonFX(RobotMap.SHOOTER_MOTOR_TWO_PDP);

        /* Factory Default all hardware to prevent unexpected behaviour */
        _motorOne.configFactoryDefault();
        _motorTwo.configFactoryDefault();

        talonCurrentLimit = new StatorCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
        CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

        _motorOne.configStatorCurrentLimit(talonCurrentLimit);
        _motorTwo.configStatorCurrentLimit(talonCurrentLimit);
  
        _motorOne.setNeutralMode(NeutralMode.Coast);
        _motorTwo.setNeutralMode(NeutralMode.Coast);

        /* Configure output and sensor direction */
        _motorOne.setInverted(_motorOneInvert);
        _motorTwo.setInverted(_motorTwoInvert);

        //set second motor as a follower
        _motorTwo.follow(_motorOne, FollowerType.PercentOutput);
        
        /* Config neutral deadband to be the smallest possible */
        _motorOne.configNeutralDeadband(0.001);

        /* Config sensor used for Primary PID [Velocity] */
        _motorOne.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                            kPIDLoopIdx, 
                                            kTimeoutMs);
                                            

        /* Config the peak and nominal outputs */
        _motorOne.configNominalOutputForward(0, kTimeoutMs);
        _motorOne.configNominalOutputReverse(0, kTimeoutMs);
        _motorOne.configPeakOutputForward(1, kTimeoutMs);
        _motorOne.configPeakOutputReverse(0, kTimeoutMs); //set so that the shooter CANNOT run backwards

        /* Config the Velocity closed loop gains in slot0 */
        _motorOne.config_kF(kPIDLoopIdx, kGains_Velocity.kF, kTimeoutMs);
        _motorOne.config_kP(kPIDLoopIdx, kGains_Velocity.kP, kTimeoutMs);
        _motorOne.config_kI(kPIDLoopIdx, kGains_Velocity.kI, kTimeoutMs);
        _motorOne.config_kD(kPIDLoopIdx, kGains_Velocity.kD, kTimeoutMs);
        /*
         * Talon FX does not need sensor phase set for its integrated sensor
         * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
         * and the user calls getSelectedSensor* to get the sensor's position/velocity.
         * 
         * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
         */
        // _motorOne.setSensorPhase(true);


        ConsolePrinter.putNumber("Shooter Velocity", () -> {return getVelocity();}, true, false);
        ConsolePrinter.putNumber("Shooter Error", () -> {return getError();}, true, false);
        ConsolePrinter.putNumber("Shooter Motor Output Percent", () -> {return _motorOne.getMotorOutputPercent();}, true, false);
        //ConsolePrinter.putNumber("Shooter Setpoint", () -> {return ticks_per_100ms_to_revs_per_minute( _motorOne.getClosedLoopTarget());}, true, false);
    }
    /**
     * Creates a new Instance of the shooter
     * @return - New shooter instance will be utilized by the commands
     */
    public static Shooter getInstance() {
        if (_instance == null) {
            _instance = new Shooter();
        }
        return _instance;
    }

    /**
     * Sets the speed (%Vbus) of the shooter motors
     * 
     * @param speed 1.0 to -1.0, positive is out of the robot
     */
    public void driveShooterMotors(double speed)
    {
        _motorOne.set(ControlMode.PercentOutput, speed);
        //driveShooterMotorTwo(speed); //Not needed this is configured as a follower
    }

    public double getVelocity()
    {
        return ticks_per_100ms_to_revs_per_minute(_motorOne.getSelectedSensorVelocity(kPIDLoopIdx));
    }

    /**
     * Sets the closed loop shooter speed.
     * 
     * @param setPoint speed in RPM
     */
    public void setSpeed(double setPoint)
    {
        setPointVelocity_sensorUnits = revs_per_minute_to_ticks_per_100ms(setPoint) ;
        _motorOne.set(ControlMode.Velocity, setPointVelocity_sensorUnits);
    }

    public double getError()
    {
        return _motorOne.getClosedLoopError(kPIDLoopIdx)/TICKS_PER_100MS;
    }

    /**
     * Converts RPM to sensor ticks per 100ms
     * 
     * @param revs speed (RPM) to convert to ticks/100ms
     */
    private double revs_per_minute_to_ticks_per_100ms(double revs) {
        return (revs / SECS_PER_MIN) * GEAR_RATIO * TICKS_PER_100MS;
    }

    /**
     * Convert speed in motor units per 100ms to RPM
     * 
     * @param ticks speed (ticks/100ms) to convert to RPM
     */
    private double ticks_per_100ms_to_revs_per_minute(double ticks) {
        //TODO: Verify conversion is correct
        return ((ticks * 10.0) / SECS_PER_MIN) / GEAR_RATIO;
    }

    public void initDefaultCommand() {
        setDefaultCommand(new DriveShooterWithJoystick());
    }
}