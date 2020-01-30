package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.team2168.RobotMap;
import org.team2168.commands.shooter.DriveShooterWithJoystick;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Shooter extends Subsystem {

    private TalonFX _motorOne;
    private TalonFX _motorTwo;

    private final boolean _motorOneReversed = false;
    private final boolean _motorTwoReversed = false;

    private static Shooter _instance;

    private Shooter() {

        _motorOne = new TalonFX(RobotMap.SHOOTER_MOTOR_ONE_PDP);
        _motorTwo = new TalonFX(RobotMap.SHOOTER_MOTOR_TWO_PDP);
    }
    /**
     * Creates a new Instance of the shooter
     * @return - New shooter instance will 
     * be utilized by the commands
     */
    public static Shooter getInstance() {
        if (_instance == null)
          _instance = new Shooter();
        return _instance;
      }
      /**
       * Used primarily for testing
       * @param speed - Will allow the speed
       * of the motor to be set
       */
    public void driveShooterMotorOne(double speed)
    {
        if(_motorOneReversed)
        {
            speed = -speed;
        }
        _motorOne.set(ControlMode.PercentOutput, speed);
    }
      /**
       * Used primarily for testing
       * @param speed - Will allow the speed
       * of the motor to be set
       */
    public void driveShooterMotorTwo(double speed)
    {
        if(_motorTwoReversed)
        {
            speed = -speed;
        }
        _motorTwo.set(ControlMode.PercentOutput, speed);
    
    }
    /**
     * Allows the motors to be set together,
     * creating a more accurate launch
     * @param speed - Allows us to set
     * the speed of the motors together
     */
    public void driveShooterMotors(double speed)
    {
        driveShooterMotorOne(speed);
        driveShooterMotorTwo(speed);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new DriveShooterWithJoystick());
    }
}