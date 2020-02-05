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

public class MoveToPositionC extends CommandGroup {
  HoodAdjust pos = HoodAdjust.getInstance();
  public MoveToPositionC() {
    switch(pos.shooterPosition){
      case POS2 :
        addSequential(new RetractShooterHood());
        addSequential(new Sleep(), 0.2);
      case POS1 : 
        addSequential(new RetractShooterHardstop());
        addSequential(new Sleep(), 0.2);
        addSequential(new ExtendShooterHood());
        addSequential(new Sleep(), 0.2);
      case POS4 :
        addSequential(new ExtendShooterHardstop());
        addSequential(new Sleep(), 0.2);
        addSequential(new RetractShooterHood());
        addSequential(new Sleep(), 0.2);
      case POS3 :
        break;
    }
    pos.setHoodPosition(HoodAdjust.HoodPosition.POS3);
  }
}
