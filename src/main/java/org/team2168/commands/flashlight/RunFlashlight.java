/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.flashlight;

import org.team2168.subsystems.Flashlight;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunFlashlight extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Flashlight flashlight;
  private double _input;
  public RunFlashlight(double input) {

    flashlight = Flashlight.getInstance();
    addRequirements(flashlight);
    this._input = input;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    flashlight.setFlashlight(_input);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    flashlight.setFlashlight(-1.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}
