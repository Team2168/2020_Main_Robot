/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.hood_adjust;

import org.team2168.commands.auto.Sleep;
import org.team2168.subsystems.HoodAdjust;



import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MoveToBenchNoShoot extends SequentialCommandGroup {
  HoodAdjust pos = HoodAdjust.getInstance();
  /**
   * Add your docs here.
   */
  public MoveToBenchNoShoot() {
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
    new ExtendShooterHood(),
    new RetractShooterHardstop(),
    new Sleep().withTimeout(0.1),
    new RetractShooterHood(),
    new Sleep().withTimeout(0.2));

      
    // addSequential(new ExtendShooterHood());
    // addSequential(new RetractShooterHardstop());
    // addSequential(new Sleep(), 0.1);
    // addSequential(new RetractShooterHood());
    // addSequential(new Sleep(), 0.2);
  }
}
