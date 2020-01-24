/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team2168.RobotMap;
import org.team2168.commands.climber_comm.DriveClimberWithJoystick;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem {
public static Climber instance = null;
public static double holdingSpeed; 
private TalonSRX climberMotor1;
private TalonSRX climberMotor2;
public Solenoid climberSolenoid;
private final boolean CLIMBER_MOTOR_1_REVERSE = false;
private final boolean CLIMBER_MOTOR_2_REVERSE = false;
public static final boolean CLIMBER_ENABLE_HIGHT_HOLD = true;
private Climber() {
  climberMotor1 = new TalonSRX(RobotMap.CLIMBER_MOTOR_1);
  climberMotor2 = new TalonSRX(RobotMap.CLIMBER_MOTOR_2);
  climberSolenoid = new Solenoid(2);
}
/** 
 *  This method will set the climbers motors to a new speed, allowing
 * the motors to be operated. 
 * @param speed - 1 is to raise the climber, and -1 is to lower it.
 */
 
public void driveClimberMotors(double speed){
  driveClimberMotor1(speed);
  driveClimberMotor2(speed);
}

/**
 * This method is mostly used for testing only motor one.
 * @param speed - 1 is to raise the climber, and -1 is to lower it.
 */
public void driveClimberMotor1(double speed){
  if(CLIMBER_MOTOR_1_REVERSE){
    speed = -speed;
  }
    
  climberMotor1.set(ControlMode.PercentOutput, speed);
}
/**
 * This method is used for testing only motor two.
 * @param speed - 1 is to raise the climber, and -1 is to lower it.
 */
public void driveClimberMotor2(double speed){
  if(CLIMBER_MOTOR_2_REVERSE){
    speed = -speed;
  }
  climberMotor2.set(ControlMode.PercentOutput, speed);
}


/**
 * This method creates a new instance of the climber, allowing other 
 * programs, i.e. the commands, to utilize it.
 * @return - Returns the new instance of the climber.
 */
public static Climber GetInstance()
{
  if (instance == null)
    instance = new Climber();
  return instance;
}

/** 
 * This method allows the ratchet to extend, preventing the climber
 * from moving from the lowered position.
 */
public void extendRatchet(){
  climberSolenoid.set(false);
}

/** 
 * This method allows the rathcet to retract, allowing the climber to move
 *  to a raised position.
 */
public void retractRatchet(){
  climberSolenoid.set(true);
}

/**
 * This will allow the program to return whether the ratchet is extended or not.
 * @return
 */
public boolean isRatchetRetracted(){
  return climberSolenoid.get();
 
}

  @Override
  /**
   * This sets the default command to drive via a joystick.
   */
  public void initDefaultCommand() {
    setDefaultCommand(new DriveClimberWithJoystick());
  }

}
