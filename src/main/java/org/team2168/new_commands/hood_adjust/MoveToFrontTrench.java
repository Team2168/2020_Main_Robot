/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.new_commands.hood_adjust;

import org.team2168.commands.hood_adjust.RetractShooterHardstop;
import org.team2168.commands.hood_adjust.RetractShooterHood;
import org.team2168.commands.shooter.DriveToXSpeed;
import org.team2168.new_commands.auto.Sleep;
import org.team2168.new_subsystems.HoodAdjust;
import org.team2168.new_subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveToFrontTrench extends CommandBase {
  HoodAdjust pos = HoodAdjust.getInstance();
  public MoveToFrontTrench() {
   //By not using a break in the switch statement, the cases will
   //follow through, allowing for less lines of code.
    // switch(pos.shooterPosition){
    //   // This code will carry the hood from the white line position 
    //   // to the wall position.
    //   case WHITE_LINE :
    //     addSequential(new ExtendShooterHood());
    //     addSequential(new Sleep(), 0.5);
    //     addSequential(new RetractShooterHardstop());
    //     addSequential(new Sleep(), 0.5);
    //   // This code will carry the hood from the wall to the
    //   // back trench position.
    //   case WALL : 
    //     addSequential(new RetractShooterHood());
    //     addSequential(new Sleep(), 0.5);
    //   // This code will complete the process and carry the hood from the
    //   // back trench position to the front trench position.
    //   case BACK_TRENCH :
    //     addSequential(new ExtendShooterHardstop());
    //     addSequential(new Sleep(), 0.5);
    //     addSequential(new ExtendShooterHood());
    //     addSequential(new Sleep(), 0.5);
    //     // In this case, the hood does not need to move.
    //   case FRONT_TRENCH :
    //     break;
    // }

    //updated---allows for any possible position, allows retract pancake under load;
    addParallel(new DriveToXSpeed(Shooter.getInstance().FRONT_TRENCH_VEL));
    addSequential(new RetractShooterHardstop());
    addSequential(new Sleep(), 0.1);
    addSequential(new RetractShooterHood());
    addSequential(new Sleep(), 2.0); //TODO tune later
    addSequential(new ExtendShooterHardstop());
    addSequential(new Sleep(), 0.1);
    addSequential(new ExtendShooterHood());
    addSequential(new Sleep(), 0.1);
    pos.setHoodPosition(HoodAdjust.HoodPosition.FRONT_TRENCH);
  }
}
