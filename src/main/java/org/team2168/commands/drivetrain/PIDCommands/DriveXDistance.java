/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;

public class DriveXDistance extends Command {
  private Drivetrain dt;
    /**target position */
    private double _targetPos;
    private double _targetAngle = 0.0;

    private static final double DEFAULT_ERROR_TOLERANCE = 0.5;

    private double _errorTolerancePosition; //0.5 inches
    private double _errorToleranceAngle = 1.0; //1.0 degree of tolerance 
    private double _loopsToSettle = 10;
    private int _withinThresholdLoops = 0;

  public DriveXDistance(double setPoint) {
    this(setPoint, DEFAULT_ERROR_TOLERANCE);
  }

  public DriveXDistance(double setPoint, double errorTolerancePosition) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    dt = Drivetrain.getInstance();
    requires(dt);

    _errorTolerancePosition = errorTolerancePosition;
    
    _targetPos = setPoint;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    dt.zeroSensors(); //heading and position
    dt.changeMaxVelAcc(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    dt.setSetPointPosition(_targetPos, _targetAngle);
    /* Check if closed loop error is within the threshld */
    if ((Math.abs(dt.getErrorPosition()) < _errorTolerancePosition) && (Math.abs(dt.getErrorHeading()) < _errorToleranceAngle)) 
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
    dt.tankDrive(0.0, 0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}