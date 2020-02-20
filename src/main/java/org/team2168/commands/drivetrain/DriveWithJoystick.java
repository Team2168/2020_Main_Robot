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
  private double heading_setpoint;
  
  public DriveWithJoystick() 
  {
    dt = Drivetrain.getInstance();
    requires(dt);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    oi = OI.getInstance();
    heading_setpoint = dt.getHeading();
    dt.switchGains(true, true);
	}

	/**
	 * Gets the joystick positions from OI and sends them to the drivetrain
	 * subsystem.
	 * 
	 * @author Liam
	 */
  @Override
  protected void execute() {
    // dt.tankDrive(oi.getGunStyleYValue()+ oi.getGunStyleXValue(),
    //   oi.getGunStyleYValue() - oi.getGunStyleXValue());

    // I think for limelight, to the right = positive, to the left = negative
    //for the gyro, to the left = pos, to the right = neg
    if(dt.isLimelightEnabled()) {
      //if limelight is enabled, command to that heading, with manual turning as an arbitrary ff
      heading_setpoint = dt.getHeading() - dt.limelight.getPos(); //TODO CHECK SIGN FOR LIMELIGHT
      dt.drive(heading_setpoint, oi.getGunStyleYValue(), oi.getGunStyleXValue());
    } else if (Math.abs(oi.getGunStyleXValue()) >= 0.01) {
      //drive straight - heading setpoint is the last heading set before turning
      dt.drive(heading_setpoint, oi.getGunStyleYValue(), oi.getGunStyleXValue());
    } else {
      //drive open loop with joysticks, store heading for future loop iterations
      heading_setpoint = dt.getHeading();
      dt.drive(oi.getGunStyleYValue(), oi.getGunStyleXValue());
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
