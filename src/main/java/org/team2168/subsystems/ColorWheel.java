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


  /**
   * Gets the current color sensed by the color sensor on the robot
   * 
   */
  public char getSensorColor() //update to have code that specifically pulls the data from the sensor into one of 4 characters
  {
    return 'B';
  }

  

  /**
  * gets the color we need to sping to from the FMS 
  * 
  * @return the first letter of the color we need to position to as a char
  */
  public char getTargetColor()
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

  /**
  * Translates what our color sensor sees into what the Color Sensor on the color wheel itself sees
  * 
  * @return the first letter of the color that the field should be reading as a char
  */

  public char getCalculatedFieldColor()
  {
    char robotSensor = getSensorColor();
    if(robotSensor == 'B')
    {
      return 'Y';
    }
    else if(robotSensor == 'G')
    {
      return 'B';
    }
    else if(robotSensor == 'R')
    {
      return 'G';
    }
    else if(robotSensor == 'Y')
    {
      return 'R';
    }
    else
    {
      return 'N';
    }
  }

  /**
  * @return the value of the inputted color as a consistent integer or the number 5 to indicate error data
  */

  public int colorWheelMapping(char inputColor)
  {
    if (inputColor == 'B') //create a method for this
    {
      return 0;
    }
    else if (inputColor == 'G')
    {
      return 1;
    }
    else if (inputColor == 'R')
    {
      return 2;
    }
    else if (inputColor == 'Y')
    {
      return 3;
    }
    else
    {
      return 5; //error return value
    }
  }

  /**
  * @return the number of degrees required to spin the color wheel in order to land on 
  * the color needed by the FMS
  */
  
  public int degToSpin()
  {
    char desiredColor = getTargetColor(); 
    char currentColor = getCalculatedFieldColor(); 
    int desiredColorNum = 0;
    int currentColorNum = 0;
    int degrees = 0;
    
    currentColorNum = colorWheelMapping(currentColor);
    if (currentColorNum == 5)
    {
      return 0;
    }
    desiredColorNum = colorWheelMapping(desiredColor);
    if (desiredColorNum == 5)
    {
      return 0;
    }

    degrees = currentColorNum - desiredColorNum;
    if (degrees == 3)
    {
      degrees = 1;
    }
    else if (degrees == -3)
    {
      degrees = -1;
    }

    return degrees * 45;
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
