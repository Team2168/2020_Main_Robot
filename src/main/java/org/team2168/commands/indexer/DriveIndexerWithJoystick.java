/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.indexer;

import org.team2168.OI;
import org.team2168.subsystems.Indexer;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveIndexerWithJoystick extends CommandBase
 {
   private Indexer _indexer;
   private OI _oi;

  public DriveIndexerWithJoystick() {
    _indexer = Indexer.getInstance();
    
    addRequirements(_indexer);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    _oi = OI.getInstance();
  }
  
  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    _indexer.drive(_oi.getIndexerJoystick());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) { 
    _indexer.drive(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
 
}
