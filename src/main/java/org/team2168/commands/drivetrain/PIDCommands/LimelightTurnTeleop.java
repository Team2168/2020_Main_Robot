/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;

public class LimelightTurnTeleop extends Command {
  private Drivetrain dt;
  /**target position */
  private double _targetPos = 0.0;

  private static final double DEFAULT_ERROR_TOLERANCE = 1.0;

  private double _errorTolerancePosition = 0.5; //0.5 inches 
  private double _errorToleranceAngle; //1.0 degree of tolerance 
  private double _loopsToSettle = 5;
  private int _withinThresholdLoops = 0;

  /**
   * Rotate the chassis at the target. Terminates when within default (1.0 deg) of tolerance
   */
  public LimelightTurnTeleop() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this(DEFAULT_ERROR_TOLERANCE);
  }

    /**
   * Rotate the chassis at the target
   * 
   * @param errorToleranceAngle allowable error tolerance (degrees) about the target angle before this command will terminate.
   */
  public LimelightTurnTeleop(double errorToleranceAngle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    dt = Drivetrain.getInstance();
    requires(dt);

    _errorToleranceAngle = errorToleranceAngle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _withinThresholdLoops = 0;
    dt.zeroSensors();
    dt.switchGains(false);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    dt.
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
    //return _withinThresholdLoops > _loopsToSettle;
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //dt.tankDrive(0.0, 0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}