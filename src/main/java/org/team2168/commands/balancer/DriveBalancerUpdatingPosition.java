/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.balancer;

import org.team2168.OI;
import org.team2168.subsystems.Balancer;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveBalancerUpdatingPosition extends CommandBase {

  private static Balancer balancer;
  private static OI oi;
  private double _setPoint;
  private boolean _readPIDFromDashboard = false;
  private double _loopsToSettle = 10;
  private int _withinThresholdLoops = 0;
  private double numRevolutions = 5.0;

  public DriveBalancerUpdatingPosition() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    balancer = Balancer.getInstance();
    addRequirements(balancer);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    oi = OI.getInstance();
    if(_readPIDFromDashboard) {
      balancer.updatePIDValues();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if(Math.abs(oi.getBalancerJoystickValue())>0.1){
      _setPoint = balancer.getPosition() + (oi.getBalancerJoystickValue() * numRevolutions);
    }
    else {
      _setPoint = balancer.getPosition();
    }

    balancer.setPositionSetPoint(_setPoint);

    /* Check if closed loop error is within the threshld */
    if (Math.abs(_setPoint-balancer.getPosition()) < balancer.getAllowedClosedLoopError()) {
      ++_withinThresholdLoops;
    } 
    else {
      _withinThresholdLoops = 0;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return _withinThresholdLoops > _loopsToSettle;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    balancer.driveMotor(0.0);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}