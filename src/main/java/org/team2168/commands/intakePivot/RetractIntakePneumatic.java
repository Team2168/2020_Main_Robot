/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*this command retracts the intake pneumatic*/
package org.team2168.commands.intakePivot;

import org.team2168.subsystems.IntakePivot;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractIntakePneumatic extends CommandBase {
  private IntakePivot intakePivot;
  public RetractIntakePneumatic() {
    intakePivot = IntakePivot.getInstance();
    addRequirements(intakePivot);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  /**
   * retracts intake pneumatic
   * @author Ian
   */
  @Override
  public void execute() {
    intakePivot.retractIntake();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public  boolean isFinished() {
    return intakePivot.isIntakeRetracted(); //checks if intake is retracted
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}