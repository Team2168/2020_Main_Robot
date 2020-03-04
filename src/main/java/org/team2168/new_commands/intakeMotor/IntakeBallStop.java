/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.new_commands.intakeMotor;

import org.team2168.commands.intakePivot.RetractIntakePneumatic;
import org.team2168.new_commands.auto.Sleep;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeBallStop extends CommandBase {

  public IntakeBallStop() {
    addParallel(new DriveIntakeWithConstant(0.3));
    addSequential(new RetractIntakePneumatic());
    addSequential(new Sleep(), 1.5);
    addSequential(new DriveIntakeWithConstant(0.0));
   
  }
}
