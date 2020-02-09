/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.intakeMotor;

import org.team2168.commands.auto.Sleep;
import org.team2168.commands.intakePivot.ExtendIntakePneumatic;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class IntakeBallStart extends CommandGroup {
  
  public IntakeBallStart() {
    addSequential(new ExtendIntakePneumatic());
    addSequential(new Sleep(), 0.0);// TO-DO: Figure out if we need a sleep command, and if so, for how long.
    addSequential(new DriveIntakeWithConstant(0.25));
  }
}
