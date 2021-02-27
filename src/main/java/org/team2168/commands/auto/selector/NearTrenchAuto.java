/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.auto.selector;

import org.team2168.Robot;
import org.team2168.commands.auto.NearTrenchAutoNoPush;
import org.team2168.commands.auto.NearTrenchAutoPush;


import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NearTrenchAuto extends CommandBase {
  public NearTrenchAuto() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    if (Robot.getPushRobot()) {
      Scheduler.getInstance().add(new NearTrenchAutoPush());
    }
  else {
    Scheduler.getInstance().add(new NearTrenchAutoNoPush());
  }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}
