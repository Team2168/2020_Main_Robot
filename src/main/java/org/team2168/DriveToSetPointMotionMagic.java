/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168;

import edu.wpi.first.wpilibj.command.Command;

public class DriveToSetPointMotionMagic extends Command {

  /**target position */
  private double _targetPos;
  private boolean _absolutePosition;
  
  int _loops = 0;  
  StringBuilder _sb;
  boolean _printStatements = false;
  private double _errorTolerance;


  
  public DriveToSetPointMotionMagic(double setPoint, boolean absolutePosition, double errorTolerance) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.exampleMotionMagicSubsystem);
    _sb = new StringBuilder();    

    _absolutePosition = absolutePosition;
    _errorTolerance = errorTolerance;
    if (_absolutePosition)
    {
      _targetPos=setPoint;
    }
    else
    {
      _targetPos = setPoint + Robot.exampleMotionMagicSubsystem._talon.getSelectedSensorPosition();
    }


  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.exampleMotionMagicSubsystem.setTalonFollower();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    /* Get current Talon SRX motor output */
    double motorOutput = Robot.exampleMotionMagicSubsystem.getMotorOutputPercent();
    
    if(_printStatements)
    {
      /* Prepare line to print */
      _sb.append("\tout:");
      /* Cast to int to remove decimal places */
      _sb.append((int) (motorOutput * 100));
      _sb.append("%");	// Percent
      _sb.append("\tVel:");
      _sb.append(Robot.exampleMotionMagicSubsystem.getVelocity());
      _sb.append("\tpos:");
      _sb.append(Robot.exampleMotionMagicSubsystem.getPosition());
      _sb.append("u"); 	// Native units
      _sb.append("\terr:");
      _sb.append(Robot.exampleMotionMagicSubsystem.getError());
      _sb.append("\ttrg:");
      _sb.append(_targetPos);

          /* Periodically print to console */
    if (++_loops >= 10) {
      _loops = 0;
      System.out.println(_sb.toString());
    }

    /* Reset created string for next loop */
    _sb.setLength(0);
    
    /* Instrumentation */
    //  Instrum.Process(_talon, _sb); 
    }

}

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.exampleMotionMagicSubsystem.reachedSetpoint(_errorTolerance);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
