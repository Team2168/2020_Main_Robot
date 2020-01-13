/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands;

import org.team2168.Robot;
import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class DriveWithJoystick extends Command 
{
  int ctrlStyle;

  /**
   * Controller Styles 0 = Tank Drive (Default) 1 = Gun Style 2 = Arcade Drive 3 =
   * GTA
   */
  private double speed;

  double rightSpeed = 0;
  double leftSpeed = 0;
  private int intCounter = 0;
  private boolean finished;
  
  public DriveWithJoystick(int inputStyle) 
  {
    requires(Robot.drivetrain);
    ctrlStyle = inputStyle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {
    intCounter = 0;
		ctrlStyle = Robot.getControlStyleInt();
		switch (ctrlStyle) {
      case 1:
			finished = false;
			Robot.drivetrain.getInstance();
		
			// reset controller
				Robot.drivetrain.resetPosition();	
				Robot.drivetrain.imu.reset();gi
				Robot.drivetrain.driveTrainPosController.reset();
        Robot.drivetrain.rotateDriveStraightController.reset();
        
      Robot.drivetrain.driveTrainPosController.setSetPoint(endDistance);
			Robot.drivetrain.driveTrainPosController.setMaxPosOutput(speed);
			Robot.drivetrain.driveTrainPosController.setMinPosOutput(-speed);


  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() 
  {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() 
  {
    Robot.drivetrain.tankDrive(0.0, 0.0);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() 
  {
    
  }
}
