/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class ColorWheel extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSolenoid pistonMove;
  private CANSparkMax spinner;
  private static ColorWheel instance = null;

  private ColorWheel()
  {
    pistonMove = new DoubleSolenoid(RobotMap.COLORWHEEL_ENGAGE_PCM,RobotMap.COLORWHEEL_DISENGAGE_PCM);
    spinner = new CANSparkMax(RobotMap.COLORWHEEL_MOTOR_PDP,MotorType.kBrushless);
  }

  public void spin(double speed)
  {
    if (RobotMap.CW_REVERSE == true)
    {
      speed = speed * -1;
    }
    spinner.set(speed);
  }

  public static ColorWheel getInstance()
  {
    if (instance == null)
    {
      instance = new ColorWheel();
    }
    return instance;
  }

  public void extendPiston()
  {
    pistonMove.set(DoubleSolenoid.Value.kForward);
  }
  public void retractPiston()
  {
    pistonMove.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean isExtended()
  {
    return pistonMove.get() == Value.kForward; 
  }





  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
