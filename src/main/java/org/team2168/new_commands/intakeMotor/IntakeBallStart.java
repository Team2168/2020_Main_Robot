/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.new_commands.intakeMotor;

import org.team2168.commands.intakePivot.ExtendIntakePneumatic;
import org.team2168.new_commands.auto.Sleep;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeBallStart extends CommandBase {
  
  public IntakeBallStart() {
    addParallel(new DriveIntakeWithConstant(0.7));//TODO set
    addSequential(new ExtendIntakePneumatic());
    addSequential(new Sleep(), 0.0);// TO-DO: Figure out if we need a sleep command, and if so, for how long.
  }
}
