/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.new_commands.hood_adjust;

import org.team2168.new_subsystems.HoodAdjust;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractShooterHardstop extends CommandBase {
  private HoodAdjust hoodAdjust;

  public RetractShooterHardstop() {
    hoodAdjust = HoodAdjust.getInstance();
    requires(hoodAdjust);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    hoodAdjust.retractPancake();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return hoodAdjust.isPancakeRetracted();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
