/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.new_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FireBallsAutoNoLineBreak extends CommandBase {
  /**
   * Add your docs here.
   */
  public FireBallsAutoNoLineBreak() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    addSequential(new WaitForShooterAtSpeed());
    addSequential(new DriveIndexerWithConstantNoStop(1.0), 0.1); 
    addParallel(new DriveIndexerWithConstant(1.0));
    addParallel(new DriveHopperWithConstant(1.0));
    addParallel(new DriveIntakeWithConstant(0.2));
  }
}
