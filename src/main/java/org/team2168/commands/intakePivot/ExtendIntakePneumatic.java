/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* this command allows the pneumatic to be extended*/
package org.team2168.commands.intakePivot;

import org.team2168.subsystems.IntakePivot;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendIntakePneumatic extends CommandBase {
  private IntakePivot intakePivot;
  public ExtendIntakePneumatic() {
    intakePivot = IntakePivot.getInstance();
    addRequirements(intakePivot); //command for intake
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  /**
   * extends intake pneumatic
   * 
   * @author Ian
   */
  @Override
  public void execute() {
    intakePivot.extendIntake();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return intakePivot.isIntakeExtended(); //checks if intake pneumatic is extended
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

  
}