/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.climber;


import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team2168.OI;
import org.team2168.Robot;
import org.team2168.subsystems.Climber;


public class DriveClimberWithJoystick extends CommandBase {
  double _speed;
  private Climber climber;
  private OI oi;
  public DriveClimberWithJoystick() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    climber = Climber.getInstance();
    addRequirements(climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    oi = OI.getInstance();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // if(climber.getPosition() > 9.0) { // && climber.getPosition() < 59.0) broke it???


      climber.driveClimberMotors(oi.getClimberJoystickValue());
    // }
    // else {
    //   climber.driveClimberMotors(0.0);
    // }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    climber.driveClimberMotors(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}


