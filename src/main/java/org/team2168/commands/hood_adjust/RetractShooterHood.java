/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.hood_adjust;

import org.team2168.subsystems.HoodAdjust;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractShooterHood extends CommandBase {
  private HoodAdjust hoodAdjust;
  public RetractShooterHood() {
    hoodAdjust = HoodAdjust.getInstance();
    addRequirements(hoodAdjust);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    hoodAdjust.retractHood();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {

    return hoodAdjust.isHoodRetracted();
    
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}
