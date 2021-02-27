/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.balancer;

import org.team2168.Robot;
import org.team2168.subsystems.Balancer;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team2168.OI;

public class DriveBalancerMotorWithJoystick extends CommandBase {
  private Balancer balancer;
  private OI oi;
  public DriveBalancerMotorWithJoystick() {
    balancer = Balancer.getInstance();
    addRequirements(balancer);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    oi = OI.getInstance();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    balancer.driveMotor(oi.getBalancerJoystickValue());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    balancer.driveMotor(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}
