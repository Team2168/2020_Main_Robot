/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.color_wheel;

import org.team2168.OI;
import org.team2168.subsystems.ColorWheel;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveColorWheelWithJoystick extends CommandBase {
  private ColorWheel colorWheel;
  private OI oi;
  private final double MAX_SPEED = 0.8;
  public DriveColorWheelWithJoystick() {
    colorWheel = ColorWheel.getInstance();
  
    addRequirements(colorWheel);
      
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    oi = OI.getInstance();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if(Math.abs(oi.getColorWheelJoystick()) < MAX_SPEED) {
      colorWheel.drive(oi.getColorWheelJoystick());
    }
    else {
      colorWheel.drive(MAX_SPEED);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    colorWheel.drive(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
 
}
