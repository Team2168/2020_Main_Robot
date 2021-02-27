/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.climber;

import org.team2168.subsystems.Climber;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetClimberPosition extends CommandBase {
  private Climber climber;
  public ResetClimberPosition() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    climber = Climber.getInstance();
    addRequirements(climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    climber.zeroEncoder();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}
