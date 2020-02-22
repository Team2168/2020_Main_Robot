/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.hood_adjust;

import org.team2168.Robot;
import org.team2168.commands.auto.Sleep;
import org.team2168.commands.shooter.DriveToXSpeed;
import org.team2168.subsystems.HoodAdjust;
import org.team2168.subsystems.Shooter;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MoveToWall extends CommandGroup {
 HoodAdjust pos = HoodAdjust.getInstance();
  public MoveToWall() {
   //By not using a break in the switch statement, the cases will
   //follow through, allowing for less lines of code.
    // switch(pos.shooterPosition){
    // // This code will carry the hood from the front trench to the
    // // back trench position.
    //   case FRONT_TRENCH :
    //     addSequential(new RetractShooterHood());
    //     addSequential(new Sleep(), 0.5);
    //     addSequential(new RetractShooterHardstop());
    //     addSequential(new Sleep(), 0.5);
    //   // Because the motions are the same, the back trench and 
    //   // white line do not need seperate sets of code
    //   case BACK_TRENCH :
    //   case WHITE_LINE : 
    //     addSequential(new ExtendShooterHood());
    //     addSequential(new Sleep(), 0.5);
    //     addSequential(new RetractShooterHardstop());
    //     addSequential(new Sleep(), 0.1);
    //   // Because the hood is already in the wall position,
    //   // no extra code is needed.
    //   case WALL :
    //     break;
    // }

    // updated---allows for any possible position, allows retract pancake under load;
    if(Robot.isPracticeBot()) {
      addParallel(new DriveToXSpeed(Shooter.getInstance().WALL_VEL_PBOT));
    }
    else {
      addParallel(new DriveToXSpeed(Shooter.getInstance().WALL_VEL)); 
    }  
    addSequential(new RetractShooterHardstop());
    addSequential(new Sleep(), 0.1);
    addSequential(new ExtendShooterHood());
    addSequential(new Sleep(), 0.1);

    pos.setHoodPosition(HoodAdjust.HoodPosition.WALL);
  }
}
