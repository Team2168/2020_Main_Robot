/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* this method allows the driver to operate the intake motor with a joystick*/
package org.team2168.commands.intake;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveWithJoystick extends Command {
  
  private static double maxSpeed = 60;
  
  public DriveWithJoystick() {
    requires(Robot.Intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.oi.getIntakeMotorJoyStick() < maxSpeed)
    {
      Robot.Intake.driveMotor(Robot.oi.getIntakeMotorJoyStick());
    }
    else
    {
      Robot.Intake.driveMotor(maxSpeed);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.Intake.driveMotor(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
