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

public class MoveToBackTrench extends CommandBase {
  HoodAdjust pos = HoodAdjust.getInstance();
  public MoveToBackTrench() {
    //By not using a break in the switch statement, the cases will
    //follow through, allowing for less lines of code.
    // switch(pos.shooterPosition){
    //   // This code will carry the hood from the white line to the wall position.
    //   case WHITE_LINE :
    //     addSequential(new ExtendShooterHood());
    //     addSequential(new Sleep(), 0.5);
    //     addSequential(new RetractShooterHardstop());
    //     addSequential(new Sleep(), 0.5);
    //   // The wall and front trench codes need to follow the same steps, 
    //   // allowing for the use of the same lines.
    //   case WALL :
    //   case FRONT_TRENCH :
    //     addSequential(new RetractShooterHood());
    //     addSequential(new Sleep(), 0.5);
    //     addSequential(new RetractShooterHardstop());
    //     addSequential(new Sleep(), 0.1);
    //   // If the hood starts at the back trench, then it does not need to move, 
    //   // meaning we just need to break; no further action is needed.
    //   case BACK_TRENCH :
    //     break;
    // }

    //updated---allows for any possible position, allows retract pancake under load;
    addParallel(new DriveToXSpeed(Shooter.getInstance().BACK_TRENCH_VEL));
    addSequential(new ExtendShooterHood());
    addSequential(new RetractShooterHardstop());
    addSequential(new Sleep(), 0.1);
    addSequential(new RetractShooterHood());
    addSequential(new Sleep(), 0.2);
    pos.setHoodPosition(HoodAdjust.HoodPosition.BACK_TRENCH);// doesn't work
  }
}

