/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.color_wheel;

import org.team2168.subsystems.ColorWheel;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveColorWheelXRotations extends Command {

  private static ColorWheel colorWheel;
  private double _setPoint;
  private boolean _readPIDFromDashboard = true;
  private double _loopsToSettle = 10;
  private int _withinThresholdLoops = 0;

  public DriveColorWheelXRotations(double setPoint) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    colorWheel = ColorWheel.getInstance();
    requires(colorWheel);
    _setPoint =setPoint;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(_readPIDFromDashboard) {
      colorWheel.updatePIDValues();
    }
    colorWheel.zeroEncoders();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    colorWheel.setPositionSetPoint(_setPoint);

    SmartDashboard.putNumber("SetPoint", _setPoint);
    SmartDashboard.putNumber("CW Velocity", colorWheel.getVelocity());
    SmartDashboard.putNumber("CW Position", colorWheel.getPosition());
    SmartDashboard.putNumber("CW Position Error", colorWheel.getPositionError());
    SmartDashboard.putNumber("CW Motor Output Percent", colorWheel.getMotorOutput());

    /* Check if closed loop error is within the threshld */
    if (Math.abs(_setPoint-colorWheel.getPosition()) < colorWheel.getAllowedClosedLoopError()) {
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