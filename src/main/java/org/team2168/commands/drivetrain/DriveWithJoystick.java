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

public class DriveWithJoystick extends Command 
{
  private Drivetrain dt;
  private OI oi;
  
  public DriveWithJoystick() 
  {
    dt = Drivetrain.getInstance();
    requires(dt);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    oi = OI.getInstance();
	}

	/**
	 * Gets the joystick positions from OI and sends them to the drivetrain
	 * subsystem.
	 * 
	 * @author Liam
	 */
  @Override
  protected void execute() {
    if ((oi.driverJoystick.getLeftStickRaw_Y() < 0.1) && (oi.driverJoystick.getLeftStickRaw_Y() > -0.1))
    {
      dt.tankDrive(oi.getGunStyleYValue(), oi.getGunStyleYValue());
    }
    else
    {
      dt.tankDrive((oi.getGunStyleYValue()) + oi.driverJoystick.getLeftStickRaw_X(),
        (oi.getGunStyleYValue()) - oi.driverJoystick.getLeftStickRaw_X());
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
    
  }
}
