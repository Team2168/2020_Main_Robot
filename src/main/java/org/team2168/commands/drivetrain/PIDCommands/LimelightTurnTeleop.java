/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.OI;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Shooter;

import edu.wpi.first.wpilibj.command.Command;

public class LimelightTurnTeleop extends Command {
  private Drivetrain dt;
  private OI oi;

  private double _targetAngle = 0.0;

  private static final double DEFAULT_ERROR_TOLERANCE = 1.0;

  private double _errorToleranceAngle; //1.0 degree of tolerance 
  private double _loopsToSettle = 5;
  private int _withinThresholdLoops = 0;
  
  private Limelight lime;

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
    oi = OI.getInstance();
    requires(dt);

    lime = Limelight.getInstance();
    requires(lime);
    _errorToleranceAngle = errorToleranceAngle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _withinThresholdLoops = 0;
    dt.zeroSensors();
    dt.switchGains(false);

    _targetAngle = 0.0;
    if(!lime.isLimelightEnabled()) {
      lime.enableLimelight(Shooter.getInstance().getFiringLocation());
    }
    lime.setLedMode(3);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double limeAngle = -lime.getPosition();

    if ((_targetAngle == 0.0) && (limeAngle != 0.0) ) {
      //Only update our target heading once per execution, but...
      //  wait until you get a non-zero value from the limelight,
      //  it takes a while to supply heading valuesafter switching pipelines

      _targetAngle = limeAngle;
    }

    dt.setSetPointHeadingTeleop(oi.getGunStyleYValue(), _targetAngle);
    /* Check if closed loop error is within the threshld */
    if (Math.abs(dt.getErrorHeading()) < _errorToleranceAngle)
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
    //dt.tankDrive(0.0, 0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}