/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.new_commands.climber;

import org.team2168.OI;
import org.team2168.new_subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveClimberWithJoystick extends CommandBase {
  double _speed;
  private Climber climber;
  private OI oi;
  private final double MAX_SPEED = 0.2;
  public DriveClimberWithJoystick() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    climber = Climber.getInstance();
    requires(climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    oi = OI.getInstance();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Math.abs(oi.getClimberJoystickValue()) < MAX_SPEED) {
      climber.driveClimberMotors(oi.getClimberJoystickValue());
    }
    else {
      climber.driveClimberMotors(MAX_SPEED);
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
    climber.driveClimberMotors(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}


