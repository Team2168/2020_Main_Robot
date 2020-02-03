/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.color_wheel;

import org.team2168.subsystems.ColorWheel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj.command.Command;

public class DriveColorWheelXRevolutions extends Command {

  private static ColorWheel colorWheel;
  private double _setPoint;
  private boolean _velocityMode;
  private boolean _readPIDFromDashboard = true;
  private double _loopsToSettle = 10;
  private int _withinThresholdLoops = 0;

  public DriveColorWheelXRevolutions(boolean velocityMode) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    colorWheel = ColorWheel.getInstance();
    requires(colorWheel);
    _velocityMode = velocityMode;
    if(_velocityMode){
      _setPoint = SmartDashboard.getNumber("Set Velocity", 0);
    }
    else {
      _setPoint = SmartDashboard.getNumber("Set Position", 0);
    }

  }

  public DriveColorWheelXRevolutions(double setPoint, boolean velocityMode) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    colorWheel = ColorWheel.getInstance();
    requires(colorWheel);
    _velocityMode = velocityMode;
    _setPoint = setPoint;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(_readPIDFromDashboard)
    {
      colorWheel.updatePIDValues();
      if(_velocityMode){
        _setPoint = SmartDashboard.getNumber("Set Velocity", 0);
      }
      else {
        _setPoint = SmartDashboard.getNumber("Set Position", 0);
      }
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    colorWheel.setSetpoint(_setPoint, _velocityMode);

    SmartDashboard.putNumber("SetPoint", _setPoint);
    SmartDashboard.putNumber("Process Variable", colorWheel.getProcessVariable());
    SmartDashboard.putNumber("Output", colorWheel.getMotorOutput());

    /* Check if closed loop error is within the threshld */
    if (Math.abs(_setPoint-colorWheel.getProcessVariable()) < colorWheel.getAllowedClosedLoopError()) {
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
    colorWheel.drive(0.0);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}