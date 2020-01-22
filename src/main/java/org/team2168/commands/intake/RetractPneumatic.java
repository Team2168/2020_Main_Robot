/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*this command retracts the intake pneumatic*/
package org.team2168.commands.intake;

import org.team2168.subsystems.Intake;

import edu.wpi.first.wpilibj.command.Command;

public class RetractPneumatic extends Command {
  private Intake intake = Intake.getInstance();
  public RetractPneumatic() {
    requires(intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  /**
   * retracts intake pneumatic
   * @author Ian
   */
  @Override
  protected void execute() {
    intake.retractIntake();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return intake.isIntakeRetracted(); //checks if intake is retracted
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end(); //for if we add things to end()
  }
}