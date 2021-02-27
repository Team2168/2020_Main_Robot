/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.indexer;

import org.team2168.Robot;
import org.team2168.subsystems.Indexer;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveIndexerWithConstantNoStop extends CommandBase { 
  private double _speed;
  private Indexer _indexer;
  public DriveIndexerWithConstantNoStop(double speed) {
    _indexer = Indexer.getInstance();
    _speed = speed;
    addRequirements(_indexer);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if(_speed != 0.0) {
      Robot.setCompressorOn(false);
    }
    else {
      Robot.setCompressorOn(true);
    }
    _indexer.drive(_speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end() {
    Robot.setCompressorOn(true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  public void interrupted() {
    end();
  }
}
