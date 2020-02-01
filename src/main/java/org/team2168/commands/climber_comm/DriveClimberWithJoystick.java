/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.climber_comm;

import edu.wpi.first.wpilibj.command.Command;
import org.team2168.OI;

import org.team2168.subsystems.Climber;


public class DriveClimberWithJoystick extends Command {
  double _speed;
  private Climber climber;
  public DriveClimberWithJoystick() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    climber = Climber.getInstance();
    requires(climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {  
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { 
    climber.driveClimberMotors(0.4*OI.getInstance().getClimberJoystickValue());

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
