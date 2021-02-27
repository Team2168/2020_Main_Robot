/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.drivetrain;

import org.team2168.commands.drivetrain.PIDCommands.DriveXDistance;
import org.team2168.commands.drivetrain.PIDCommands.TurnXAngle;



import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FirstPath extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public FirstPath() {
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
    addCommands(
      new TurnXAngle(36.0),
      new DriveXDistance(-100.0, 0.5), //-111.52
      new TurnXAngle(-36.0)
    );
    
    // addSequential(new TurnXAngle(36.0));
    // addSequential(new DriveXDistance(-100.0, 0.5)); //-111.52
    // addSequential(new TurnXAngle(-36.0));
    // addParallel(new IntakeBallStart());
    // addSequential(new DriveXDistance(-162.0, 0.25));
    // addSequential(new DriveXDistance(162.0, 0.25));
    // addParallel(new IntakeBallStop());



  }
}
