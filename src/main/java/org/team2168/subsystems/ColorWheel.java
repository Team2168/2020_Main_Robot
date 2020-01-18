/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

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

  private final boolean COLOR_WHEEL_MOTOR_REVERSE = false;
  private DoubleSolenoid colorWheelExtender;
  private CANSparkMax colorWheelMotor;
  private static ColorWheel instance = null;

  private ColorWheel()
  {
    colorWheelExtender = new DoubleSolenoid(RobotMap.COLORWHEEL_ENGAGE_PCM,RobotMap.COLORWHEEL_DISENGAGE_PCM);
    colorWheelMotor = new CANSparkMax(RobotMap.COLORWHEEL_MOTOR_PDP,MotorType.kBrushless);
    colorWheelMotor.setSmartCurrentLimit(60);
    colorWheelMotor.setControlFramePeriodMs(20);
    colorWheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
    colorWheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    colorWheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
  }

  public void drive(double speed)
  {
    if (COLOR_WHEEL_MOTOR_REVERSE)
    {
      speed = -speed;
    }
    colorWheelMotor.set(speed);
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
    colorWheelExtender.set(DoubleSolenoid.Value.kForward);
  }
  public void retractPiston()
  {
    colorWheelExtender.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean isExtended()
  {
    return colorWheelExtender.get() == Value.kForward; 
  }

  public boolean isRetracted()
  {
    return colorWheelExtender.get() == Value.kReverse; 
  }




  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
