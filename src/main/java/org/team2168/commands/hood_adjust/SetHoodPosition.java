/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.hood_adjust;

import org.team2168.subsystems.HoodAdjust;
import org.team2168.subsystems.HoodAdjust.HoodPosition;

import edu.wpi.first.wpilibj.command.Command;

public class SetHoodPosition extends Command {
  private HoodAdjust hoodAdjust;
  private HoodPosition _pos;
  public SetHoodPosition(HoodPosition pos) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    hoodAdjust = HoodAdjust.getInstance();
    requires(hoodAdjust);
    _pos = pos;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    hoodAdjust.setHoodPosition(_pos);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return _pos == hoodAdjust.getHoodPosition();
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
