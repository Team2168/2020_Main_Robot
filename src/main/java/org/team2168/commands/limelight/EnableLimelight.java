/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.limelight;

import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.HoodAdjust;
import org.team2168.subsystems.Limelight;
import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class EnableLimelight extends Command {
  private Limelight limelight;
  private HoodAdjust hoodPos; 
  public EnableLimelight() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    limelight = Limelight.getInstance();
    hoodPos = HoodAdjust.getInstance();
    requires(limelight);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    limelight.enableLimelight(hoodPos.getHoodPosition());
    System.out.println("limelight enabled");
    System.out.println(hoodPos.getHoodPosition());
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
