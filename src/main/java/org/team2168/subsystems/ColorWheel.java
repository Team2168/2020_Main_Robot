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

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DriverStation;


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

  public char getSensorColor() //update to have code that specifically pulls the data from the sensor into one of 4 characters
  {
    return 'B';
  }

  public char getFieldColor()
  {
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
        case 'B' :
         return 'B';
       case 'G' :
          return 'G';
        case 'R' :
          return 'R';
        case 'Y' :
          return 'Y';
        default :
          return 'N';
      }
    }
    else {
      return 'N';
    } 
  }

  public int degToSpin()
  {
    char desiredColor = getFieldColor();
    char currentColor = getSensorColor();
    int valueToBeReturned = 1;
    if (currentColor == 'B')
    {
      if (desiredColor == 'B')
      {
        valueToBeReturned = 90;
      }
      else if (desiredColor == 'G')
      {
        valueToBeReturned = 135;
      }
      else if (desiredColor == 'R')
      {
        valueToBeReturned = 0;
      }
      else if (desiredColor == 'Y')
      {
        valueToBeReturned = 45;
      }
      else
      {
        valueToBeReturned = 1;
      }
    }
    else if (currentColor == 'Y')
    {
      if (desiredColor == 'B')
      {
        valueToBeReturned = 135;
      }
      else if (desiredColor == 'G')
      {
        valueToBeReturned = 0;
      }
      else if (desiredColor == 'R')
      {
        valueToBeReturned = 45;
      }
      else if (desiredColor == 'Y')
      {
        valueToBeReturned = 90;
      }
      else
      {
        valueToBeReturned = 1;
      }
    }
    else if (currentColor == 'G')
    {
      if (desiredColor == 'B')
      {
        valueToBeReturned = 45;
      }
      else if (desiredColor == 'G')
      {
        valueToBeReturned = 90;
      }
      else if (desiredColor == 'R')
      {
        valueToBeReturned = 135;
      }
      else if (desiredColor == 'Y')
      {
        valueToBeReturned = 0;
      }
    }
    else if (currentColor == 'R')
    {
      if (desiredColor == 'B')
      {
        valueToBeReturned = 0;
      }
      else if (desiredColor == 'G')
      {
        valueToBeReturned = 45;
      }
      else if (desiredColor == 'R')
      {
        valueToBeReturned = 90;
      }
      else if (desiredColor == 'Y')
      {
        valueToBeReturned = 135;
      }
    }
    else
    {
      valueToBeReturned = 1; //error return value
    }
    return valueToBeReturned;
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
  }
}
