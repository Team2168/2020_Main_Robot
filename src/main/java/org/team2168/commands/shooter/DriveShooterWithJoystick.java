/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.shooter;


import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team2168.OI;
import org.team2168.subsystems.Shooter;


public class DriveShooterWithJoystick extends CommandBase {
  private Shooter shooter;
  private OI oi;
  public DriveShooterWithJoystick() {
    shooter = Shooter.getInstance();
    oi = OI.getInstance();
    addRequirements(shooter);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    shooter.driveShooterMotors(oi.getShooterJoystick());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    shooter.driveShooterMotors(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}
