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
import org.team2168.utils.LinearInterpolator;

import edu.wpi.first.wpilibj.command.Command;

public class DriveWithJoystickLimelight extends Command 
{
  private Drivetrain dt;
  private Limelight limelight;
  private OI oi;
  private LinearInterpolator limeLightInterpolator;
  private static final double LIMELIGHT_ERROR_TOLERANCE = 1.0;
  private double[][] limelightArray = { 
    { -27.0, 0.2}, //limelight offset, turning gain
    { -LIMELIGHT_ERROR_TOLERANCE, 0.07}, 
    { LIMELIGHT_ERROR_TOLERANCE, -0.07}, 
    { 27.0, -0.2}
  };
  private double limelight_offset;
  private double turnGain;
  
  public DriveWithJoystickLimelight() 
  {
    dt = Drivetrain.getInstance();
    limelight = Limelight.getInstance();
    requires(dt);
    limeLightInterpolator = new LinearInterpolator(limelightArray);

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
    limelight_offset = limelight.getPosition();
    if(limelight.isLimelightEnabled()) {
      if(limelight_offset > LIMELIGHT_ERROR_TOLERANCE) {
        turnGain = limeLightInterpolator.interpolate(limelight_offset) + oi.getGunStyleXValue();
      }
      else {
        turnGain = oi.getGunStyleXValue();
      }
    }
    else {
      turnGain = oi.getGunStyleXValue();
    }
    // dt.drive(oi.getGunStyleYValue(), turnGain);
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
