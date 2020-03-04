/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.new_commands.drivetrain;

import org.team2168.new_commands.drivetrain.PIDCommands.DriveXDistance;
import org.team2168.new_commands.drivetrain.PIDCommands.TurnXAngle;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FirstPathReverse extends CommandBase {
  /**
   * Add your docs here.
   */
  public FirstPathReverse() {
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
    addSequential(new TurnXAngle(36.0));
    addSequential(new DriveXDistance(104.0)); //-111.52
    addSequential(new TurnXAngle(-36.0));
  }
}
