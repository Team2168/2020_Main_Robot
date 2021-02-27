/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.drivetrain;

import org.team2168.OI;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Limelight;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveWithJoystick extends CommandBase 
{
  private Drivetrain dt;
  private Limelight lime;
  private OI oi;
  
  public DriveWithJoystick() 
  {
    dt = Drivetrain.getInstance();
    addRequirements(dt);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    oi = OI.getInstance();
    lime = Limelight.getInstance();
    lime.setLedMode(1);
	}

	/**
	 * Gets the joystick positions from OI and sends them to the drivetrain
	 * subsystem.
	 * 
	 * @author Liam
	 */
  @Override
  public void execute() {
    dt.tankDrive(oi.getGunStyleYValue()+ oi.getGunStyleXValue(),
      oi.getGunStyleYValue() - oi.getGunStyleXValue());
  }

  // Called repeatedly when this Command is scheduled to run
  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0.0, 0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}
