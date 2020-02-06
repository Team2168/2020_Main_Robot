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

public class MoveToFrontTrench extends CommandGroup {
  HoodAdjust pos = HoodAdjust.getInstance();
  public MoveToFrontTrench() {
    switch(pos.shooterPosition){
      case WHITE_LINE :
        addSequential(new ExtendShooterHood());
        addSequential(new Sleep(), 0.2);
      case WALL : 
        addSequential(new RetractShooterHardstop());
        addSequential(new Sleep(), 0.2);
        addSequential(new RetractShooterHood());
        addSequential(new Sleep(), 0.2);
      case BACK_TRENCH :
        addSequential(new ExtendShooterHardstop());
        addSequential(new Sleep(), 0.2);
        addSequential(new ExtendShooterHood());
        addSequential(new Sleep(), 0.2);
      case FRONT_TRENCH :
        break;
    }
    pos.setHoodPosition(HoodAdjust.HoodPosition.FRONT_TRENCH);
  }
}
