/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.balancer;

import org.team2168.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team2168.subsystems.Balancer;


public class DriveBalancerWithConstant extends CommandBase {

  private double _speed;
  private Balancer balancer;

  public DriveBalancerWithConstant(double speed) {
    // Use requires() here to declare subsystem dependencies
    balancer = Balancer.getInstance();
    _speed = speed;
    addRequirements(balancer);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    balancer.driveMotor(_speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    balancer.driveMotor(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}
