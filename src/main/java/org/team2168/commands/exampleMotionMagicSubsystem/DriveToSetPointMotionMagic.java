/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.exampleMotionMagicSubsystem;

import org.team2168.subsystems.ExampleMotionMagicSubsystem;

import edu.wpi.first.wpilibj.command.Command;

public class DriveToSetPointMotionMagic extends Command {

  /**target position */
  private double _targetPos;
  private boolean _absolutePosition;
  
  private double _errorTolerance;
  private double _loopsToSettle = 10;
  private int _withinThresholdLoops = 0;


  
  public DriveToSetPointMotionMagic(double setPoint, boolean absolutePosition, double errorTolerance) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(ExampleMotionMagicSubsystem.getInstance());

    _absolutePosition = absolutePosition;
    _errorTolerance = errorTolerance;
    if (!_absolutePosition)
    {
      ExampleMotionMagicSubsystem.getInstance().zeroEncoder();
    }
    _targetPos = setPoint;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    ExampleMotionMagicSubsystem.getInstance().setSetPoint(_targetPos);
    /* Check if closed loop error is within the threshld */
    if (Math.abs(ExampleMotionMagicSubsystem.getInstance().getErrorPosition()) < _errorTolerance) {
      ++_withinThresholdLoops;
    } 
    else {
      _withinThresholdLoops = 0;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return _withinThresholdLoops > _loopsToSettle;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    ExampleMotionMagicSubsystem.getInstance().drive(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
