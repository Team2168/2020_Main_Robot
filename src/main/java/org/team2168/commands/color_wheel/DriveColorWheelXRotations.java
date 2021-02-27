/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.color_wheel;

import org.team2168.subsystems.ColorWheel;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveColorWheelXRotations extends CommandBase {

  private static ColorWheel colorWheel;
  private double _setPoint;
  private boolean _readPIDFromDashboard = false;
  private double _loopsToSettle = 10;
  private int _withinThresholdLoops = 0;
  private int counter = 0;
  private double _targetPosition;

  public DriveColorWheelXRotations(double setPoint) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    colorWheel = ColorWheel.getInstance();
    addRequirements(colorWheel);
    _setPoint =setPoint;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    counter = 0;

    if(_readPIDFromDashboard) {
      colorWheel.updatePIDValues();
    }
    _withinThresholdLoops = 0;
    _targetPosition = _setPoint + colorWheel.getPosition();
    //System.out.println(_setPoint + " " + colorWheel.getPosition());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    counter++;
    colorWheel.setPositionSetPoint(_targetPosition);
    SmartDashboard.putNumber("SetPoint", _setPoint);
    SmartDashboard.putNumber("CW Velocity", colorWheel.getVelocity());
    SmartDashboard.putNumber("CW Position", colorWheel.getPosition());
    SmartDashboard.putNumber("CW Position Error", colorWheel.getPositionError());
    SmartDashboard.putNumber("CW Motor Output Percent", colorWheel.getMotorOutput());

    /* Check if closed loop error is within the threshld */
    if (Math.abs(colorWheel.getPositionError()) < colorWheel.getAllowedClosedLoopError()) {
      ++_withinThresholdLoops;
    } 
    else {
      _withinThresholdLoops = 0;
    }
    // System.out.println(colorWheel.getPositionError() + " " + counter);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return _withinThresholdLoops > _loopsToSettle;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    colorWheel.drive(0.0);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}