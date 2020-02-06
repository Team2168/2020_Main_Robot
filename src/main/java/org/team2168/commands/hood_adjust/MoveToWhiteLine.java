/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.hood_adjust;

import org.team2168.commands.auto.Sleep;
import org.team2168.subsystems.HoodAdjust;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MoveToWhiteLine extends CommandGroup {
HoodAdjust pos = HoodAdjust.getInstance();
  public MoveToWhiteLine() {
    switch(pos.shooterPosition){
      case FRONT_TRENCH :
        addSequential(new RetractShooterHood());
        addSequential(new Sleep(), 0.2);
      case BACK_TRENCH :
        addSequential(new RetractShooterHardstop());
        addSequential(new Sleep(), 0.2);
        addSequential(new ExtendShooterHood());
        addSequential(new Sleep(), 0.2);
      case WALL :
        addSequential(new ExtendShooterHardstop());
        addSequential(new Sleep(), 0.2);
        addSequential(new RetractShooterHood());
        addSequential(new Sleep(), 0.1);
      case WHITE_LINE :
        break;
    }
   pos.setHoodPosition(HoodAdjust.HoodPosition.WHITE_LINE);
  }
}
