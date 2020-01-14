/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems.climber;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.team2168.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//import edu.wpi.first.wpilibj.DigitalInput;
/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
public static final double holdingSpeed;
private TalonSRX climberMotor1;
private TalonSRX climberMotor2;
//private static DigitalInput isOneRaised;
//private static DigitalInput isTwoRaised;
private Climber() {
  climberMotor1 = new TalonSRX(RobotMap.CLIMBER_MOTOR_1);
  climberMotor2 = new TalonSRX(RobotMap.CLIMBER_MOTOR_2);
}

public void adjustClimber(double speed){
  climberMotor1.set(ControlMode.PercentOutput, speed);
  climberMotor2.set(ControlMode.PercentOutput, speed);
}

//public void lowerClimber(double speed){
  //climberMotor2.set(-speed);
  //climberMotor2.set(-speed);
//}
//public boolean isOneFullyRaised(){
  //return isOneRaised();
//}
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
