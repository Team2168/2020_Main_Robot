/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.color_wheel_pivot;

import org.team2168.subsystems.ColorWheelPivot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DisengageColorWheel extends CommandBase {
  private ColorWheelPivot colorWheelPivot;
  public DisengageColorWheel() {
    colorWheelPivot = ColorWheelPivot.getInstance();
    addRequirements(colorWheelPivot);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    colorWheelPivot.retractPiston();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return colorWheelPivot.isRetracted();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}
