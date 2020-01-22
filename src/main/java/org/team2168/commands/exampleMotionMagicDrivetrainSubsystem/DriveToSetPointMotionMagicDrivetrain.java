 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.exampleMotionMagicDrivetrainSubsystem;

import org.team2168.subsystems.ExampleMotionMagicDrivetrainSubsystem;

import edu.wpi.first.wpilibj.command.Command;

public class DriveToSetPointMotionMagicDrivetrain extends Command {

  /**target position */
  private double _targetPos;
  private double _targetAngle;
  private boolean _absolutePosition;
  
  private double _errorTolerancePosition;
  private double _errorToleranceAngle;
  private double _loopsToSettle = 10;
  private int _withinThresholdLoops = 0;

  public DriveToSetPointMotionMagicDrivetrain(double setPoint, boolean absolutePosition) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(ExampleMotionMagicDrivetrainSubsystem.getInstance());

    _absolutePosition = absolutePosition;
    _errorTolerancePosition = 0.5; //0.5 inches TODO need to figure out conversion
    _errorToleranceAngle = 1.0; //1.0 degree of tolerance TODO need to figure out conversion

    if (!_absolutePosition)
    {
      ExampleMotionMagicDrivetrainSubsystem.getInstance().zeroSensors();
    }
    _targetPos = setPoint;
  }
  
  public DriveToSetPointMotionMagicDrivetrain(double setPoint, boolean absolutePosition, double errorTolerancePosition, double errorToleranceAngle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(ExampleMotionMagicDrivetrainSubsystem.getInstance());

    _absolutePosition = absolutePosition;
    _errorTolerancePosition = errorTolerancePosition;
    _errorToleranceAngle = errorToleranceAngle;

    if (!_absolutePosition)
    {
      ExampleMotionMagicDrivetrainSubsystem.getInstance().zeroSensors();
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
    ExampleMotionMagicDrivetrainSubsystem.getInstance().setSetPoint(_targetPos, _targetAngle);
    /* Check if closed loop error is within the threshld */
    if (Math.abs((ExampleMotionMagicDrivetrainSubsystem.getInstance().getErrorPosition())) < _errorTolerancePosition && (Math.abs(ExampleMotionMagicDrivetrainSubsystem.getInstance().getErrorHeading()) < _errorToleranceAngle)) 
    {
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
    ExampleMotionMagicDrivetrainSubsystem.getInstance().drive(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
