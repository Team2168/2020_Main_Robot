/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.LEDs;

import org.team2168.Robot;
import org.team2168.subsystems.LEDs;
import org.team2168.subsystems.ColorWheel;

import edu.wpi.first.wpilibj.command.Command;

public class DefaultTeleopPattern extends Command {
  private LEDs leds;
  private ColorWheel colorWheel;
  public DefaultTeleopPattern() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    leds = LEDs.getInstance();
    colorWheel = ColorWheel.getInstance();
    requires(leds);
    requires(colorWheel);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    char fmsColor = colorWheel.getTargetColor();
    char fieldSensor = colorWheel.getCalculatedFieldColor();

    // Displays the current color that the sensor on the Color Wheel should be seeing
    if (fieldSensor == 'R')
    {
      leds.writePatternOneColor(leds.PATTERN_SOLID, 255, 0, 0);
    }
    else if (fieldSensor == 'G')
    {
      leds.writePatternOneColor(leds.PATTERN_SOLID, 0, 0, 255);
    } 
    else if (fieldSensor == 'B')
    {
      leds.writePatternOneColor(leds.PATTERN_SOLID, 0, 255, 0);
    } 
    else if (fieldSensor == 'Y')
    {
      leds.writePatternOneColor(leds.PATTERN_SOLID, 255, 0, 255);
    }
    // Displays the target color for position control
    else if (fmsColor == 'R')
    {
      leds.writePatternOneColor(leds.PATTERN_SOLID, 255, 0, 0);
    }
    else if (fmsColor == 'G')
    {
      leds.writePatternOneColor(leds.PATTERN_SOLID, 0, 0, 255);
    } 
    else if (fmsColor == 'B')
    {
      leds.writePatternOneColor(leds.PATTERN_SOLID, 0, 255, 0);
    } 
    else if (fmsColor == 'Y')
    {
      leds.writePatternOneColor(leds.PATTERN_SOLID, 255, 0, 255);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
