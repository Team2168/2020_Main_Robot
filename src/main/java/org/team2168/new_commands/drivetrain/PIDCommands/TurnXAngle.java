/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.new_commands.drivetrain.PIDCommands;

import org.team2168.new_subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnXAngle extends CommandBase {
  private Drivetrain dt;
  /**target position */
  private double _targetPos = 0.0;
  private double _targetAngle;

  private static final double DEFAULT_ERROR_TOLERANCE = 1.0;

  private double _errorTolerancePosition = 0.5; //0.5 inches 
  private double _errorToleranceAngle; //1.0 degree of tolerance 
  private double _loopsToSettle = 10;
  private int _withinThresholdLoops = 0;

  /**
   * positive turns left
   */
  public TurnXAngle(double setPoint) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this(setPoint, DEFAULT_ERROR_TOLERANCE);
  }

  public TurnXAngle(double setPoint, double errorToleranceAngle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    dt = Drivetrain.getInstance();
    requires(dt);

    _errorToleranceAngle = errorToleranceAngle;
    _targetAngle = setPoint;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    dt.zeroSensors();
    dt.switchGains(false);

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