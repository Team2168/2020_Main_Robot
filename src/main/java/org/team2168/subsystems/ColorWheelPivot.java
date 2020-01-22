/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class ColorWheelPivot extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSolenoid colorWheelExtender;
  private static ColorWheelPivot instance = null;

  private ColorWheelPivot()
  {
    colorWheelExtender = new DoubleSolenoid(RobotMap.COLORWHEEL_ENGAGE_PCM,RobotMap.COLORWHEEL_DISENGAGE_PCM);
  }

    /**
   * 
   * @return an instance of the ColorWheel subsystem. 
   */

  public static ColorWheelPivot getInstance()
  {
    if (instance == null)
    {
      instance = new ColorWheelPivot();
    }
    return instance;
  }

  /**
   * activates the DoubleSolenoid (goes up)
   */
  public void extendPiston()
  {
    colorWheelExtender.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * deactivates the DoubleSolenoid (goes down)
   */
  public void retractPiston()
  {
    colorWheelExtender.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * 
   * @return an activated value if not extended
   */
  public boolean isExtended()
  {
    return colorWheelExtender.get() == Value.kForward; 
  }

  /**
   * 
   * @return an deactivated value if extended
   */
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
