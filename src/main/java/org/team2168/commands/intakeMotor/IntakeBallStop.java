/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.intakeMotor;

import org.team2168.commands.auto.Sleep;
import org.team2168.commands.intakePivot.RetractIntakePneumatic;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeBallStop extends ParallelCommandGroup {

  public IntakeBallStop() {
    addCommands(
      new DriveIntakeWithConstant(0.3),
      new SequentialCommandGroup(new RetractIntakePneumatic(),
                                 new Sleep().withTimeout(1.5),
                                 new DriveIntakeWithConstant(0.0))
      );
    
    //   addParallel(new DriveIntakeWithConstant(0.3));
    // addSequential(new RetractIntakePneumatic());
    // addSequential(new Sleep(), 1.5);
    // addSequential(new DriveIntakeWithConstant(0.0));
   
  }
}
