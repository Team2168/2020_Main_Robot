/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.hopper;

import org.team2168.subsystems.Hopper;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveHopperWithConstant extends CommandBase {
  private Hopper hopper;
  private double speed;

  public DriveHopperWithConstant(double speed) {
    hopper = Hopper.getInstance();
    addRequirements(hopper);

    this.speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    hopper.drive(speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    hopper.drive(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
 
}
