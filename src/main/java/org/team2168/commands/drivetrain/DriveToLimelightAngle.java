/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.drivetrain;

import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.HoodAdjust;
import org.team2168.subsystems.Limelight;

import edu.wpi.first.wpilibj.command.Command;

public class DriveToLimelightAngle extends Command {
  private Drivetrain dt;
  private Limelight limelight;
  private HoodAdjust hoodPos;
  /**target position */
  private double _targetPos = 0.0;
  private double _targetAngle;

  private static final double DEFAULT_ERROR_TOLERANCE = 1.0;

  private double _errorTolerancePosition = 0.5; //0.5 inches 
  private double _errorToleranceAngle; //1.0 degree of tolerance 
  private double _loopsToSettle = 10;
  private int _withinThresholdLoops = 0;

  public DriveToLimelightAngle() {
    this(DEFAULT_ERROR_TOLERANCE);
  }
  public DriveToLimelightAngle(double errorToleranceAngle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    dt = Drivetrain.getInstance();
    limelight = Limelight.getInstance();
    hoodPos = HoodAdjust.getInstance();
    requires(dt);
    requires(limelight);

    _errorToleranceAngle = errorToleranceAngle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _withinThresholdLoops = 0;
    dt.zeroSensors();
    dt.switchGains(false);
    limelight.enableLimelight(hoodPos.getHoodPosition());
    _targetAngle = dt.getHeading() - limelight.getPosition();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    _targetAngle = dt.getHeading() - limelight.getPosition();
    dt.setSetPointPosition(_targetPos, _targetAngle);

          /* Check if closed loop error is within the threshld */
    if ((Math.abs(dt.getErrorPosition()) < _errorTolerancePosition) && (Math.abs(dt.getErrorHeading()) < _errorToleranceAngle)) 
    {
      ++_withinThresholdLoops;
    } 
    else {
      _withinThresholdLoops = 0;
    }
    System.out.println(_targetAngle);
    
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