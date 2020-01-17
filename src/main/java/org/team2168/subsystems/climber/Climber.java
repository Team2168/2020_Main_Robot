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

import edu.wpi.first.wpilibj.command.Subsystem;

//import edu.wpi.first.wpilibj.DigitalInput;
/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
public static Climber instance = null;
public static double holdingSpeed; 
private TalonSRX climberMotor1;
private TalonSRX climberMotor2;
private final boolean CLIMBER_MOTOR_1_REVERSE = false;
private final boolean CLIMBER_MOTOR_2_REVERSE = false;
//private static DigitalInput isOneRaised;
//private static DigitalInput isTwoRaised;
private Climber() {
  climberMotor1 = new TalonSRX(RobotMap.CLIMBER_MOTOR_1);
  climberMotor2 = new TalonSRX(RobotMap.CLIMBER_MOTOR_2);
}

public void driveClimberMotors(double speed){
  driveClimberMotor1(speed);
  driveClimberMotor2(speed);
}

public void driveClimberMotor1(double speed){
  if(CLIMBER_MOTOR_1_REVERSE){
    speed = -speed;
  climberMotor1.set(ControlMode.PercentOutput, speed);
  }
}

public void driveClimberMotor2(double speed){
  if(CLIMBER_MOTOR_2_REVERSE){
    speed = -speed;
  }
  climberMotor2.set(ControlMode.PercentOutput, speed);
}


public static Climber GetInstance()
{
  if (instance == null)
    instance = new Climber();
  return instance;
  
    
}


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveClimberWithJoystick());
  }

public double getPotPos() {
	return 0;
}
}
