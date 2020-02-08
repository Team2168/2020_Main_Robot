/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.drivetrain;

import org.team2168.OI;
import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;

public class DriveWithJoystickLimelight extends Command 
{
  private Drivetrain dt;
  private OI oi;
  private double limelight_offset;
  
  public DriveWithJoystickLimelight() 
  {
    dt = Drivetrain.getInstance();
    requires(dt);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    oi = OI.getInstance();
    dt.setUpVelocityPID();
	}

	/**
	 * Gets the joystick positions from OI and sends them to the drivetrain
	 * subsystem.
	 * 
	 * @author Liam
	 */
  @Override
  protected void execute() {
    // limelight_offset = dt.limelight.getPos();
    dt.zeroPigeon();
    if(oi.operatorJoystick.getLeftStickRaw_Y()> 0.1){
      dt.setSetPointVelocity((oi.operatorJoystick.getLeftStickRaw_Y()*dt.getMaxVel()), 0.0);
    }
    else {
      dt.setSetPointVelocity(0.0, 5.0);
    }

  }

  // Called repeatedly when this Command is scheduled to run
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
