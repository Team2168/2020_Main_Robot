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
import org.team2168.commands.color_wheel.DriveColorWheelWithJoystick;

import edu.wpi.first.wpilibj.command.Subsystem;


public class ColorWheel extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final boolean COLOR_WHEEL_MOTOR_REVERSE = false;
  private CANSparkMax colorWheelMotor;
  private static ColorWheel instance = null;

  private ColorWheel()
  {
    colorWheelMotor = new CANSparkMax(RobotMap.COLORWHEEL_MOTOR_PDP,MotorType.kBrushless);
    colorWheelMotor.setSmartCurrentLimit(30);
    colorWheelMotor.setControlFramePeriodMs(20);
    colorWheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
    colorWheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    colorWheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
  }

  /**
   * Sets the speed of the motor
   * 
   * @param speed positive is right, negative is left, 0 is hold still
   */
  public void drive(double speed)
  {
    if (COLOR_WHEEL_MOTOR_REVERSE)
    {
      speed = -speed;
    }
    colorWheelMotor.set(speed);
  }

  /**
   * 
   * @return an instance of the ColorWheel subsystem. 
   */
  public static ColorWheel getInstance()
  {
    if (instance == null)
    {
      instance = new ColorWheel();
    }
    return instance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveColorWheelWithJoystick());
  }
}
