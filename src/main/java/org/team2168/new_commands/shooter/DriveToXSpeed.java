/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.new_commands.shooter;

import org.team2168.new_subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToXSpeed extends CommandBase {

  private Shooter shooter;
  /**target position */
  private double _targetVelocity;

public DriveToXSpeed(double setPoint) {
  // Use requires() here to declare subsystem dependencies
  // eg. requires(chassis);
  shooter = Shooter.getInstance();
  requires(shooter);
  this._targetVelocity = setPoint;
}


// Called just before this Command runs the first time
@Override
protected void initialize() {
}

// Called repeatedly when this Command is scheduled to run
@Override
protected void execute() {
  shooter.setSpeed(_targetVelocity);
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
  end();
}
}