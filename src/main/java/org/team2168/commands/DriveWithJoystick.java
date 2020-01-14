/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands;

import org.team2168.OI;
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
  //private double speed;

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
        // reset controller
          Robot.drivetrain.imu.reset();
          //Robot.drivetrain.driveTrainPosController.reset();
          //Robot.drivetrain.rotateDriveStraightController.reset();
          
        //Robot.drivetrain.driveTrainPosController.setMaxPosOutput(speed);
        // Robot.drivetrain.driveTrainPosController.setMinPosOutput(-speed);
        // Robot.drivetrain.driveTrainPosController.Enable();
        //Robot.drivetrain.rotateDriveStraightController.Enable();

        System.out.println("Initialize case ran");
		  default:
			break;
		}
	}

	/**
	 * Gets the joystick positions from OI and sends them to the drivetrain
	 * subsystem.
	 * 
	 * @author Liam
	 */
  @Override
  protected void execute() {
    //double headingCorrection = 0.0;
    ctrlStyle = Robot.getControlStyleInt();

    switch (ctrlStyle) {
    /**
     * Tank Drive
     */
    case 0:
      Robot.drivetrain.driveLeft(OI.getInstance().driverJoystick.getLeftStickRaw_Y());
      Robot.drivetrain.driveRight(OI.getInstance().driverJoystick.getRightStickRaw_Y());
      break;

    /**
     * Gun Style Controller
     */
    // X Values
    // full in: -0.516
    // nothing: 0.354 & 0.342
    // full out: 0.622
    case 1: {
      // lastRotateOutput =
      // Robot.drivetrain.rotateDriveStraightController.getControlOutput();
      // headingCorrection =
      // (Robot.drivetrain.rotateDriveStraightController.getControlOutput());

      Robot.drivetrain.tankDrive(
          (OI.getInstance().getGunStyleYValue()) + OI.getInstance().driverJoystick.getLeftStickRaw_X(),
          (OI.getInstance().getGunStyleYValue()) - OI.getInstance().driverJoystick.getLeftStickRaw_X());
    }

      break;

    /**
     * Arcade Drive
     */
    case 2:
      Robot.drivetrain.driveLeft(
          OI.getInstance().driverJoystick.getLeftStickRaw_Y() + OI.getInstance().driverJoystick.getRightStickRaw_X());
      Robot.drivetrain.driveRight(
          OI.getInstance().driverJoystick.getLeftStickRaw_Y() - OI.getInstance().driverJoystick.getRightStickRaw_X());
      break;
    /**
     * GTA Drive
     */
    case 3:
      double fwdSpeed = OI.getInstance().driverJoystick.getRightTriggerAxisRaw();
      double revSpeed = OI.getInstance().driverJoystick.getLeftTriggerAxisRaw();
      double speed = fwdSpeed - revSpeed;
      double rotation = OI.getInstance().driverJoystick.getRightStickRaw_X();

      // Adjusts angle while moving
      if (speed != 0 && rotation != 0) {
        Robot.drivetrain.driveLeft(rotation * speed);
        Robot.drivetrain.driveRight(-rotation * speed);
      }
      // Allows Robot to spin in place without needing to press in triggers
      else if (speed == 0 && rotation != 0) {
        Robot.drivetrain.driveLeft(rotation);
        Robot.drivetrain.driveRight(-rotation);
      }
      // Allows Robot to drive straight
      else if (speed != 0 && rotation == 0) {
        Robot.drivetrain.driveLeft(speed);
        Robot.drivetrain.driveRight(speed);
      }
      break;

    /**
     * New Gun Style Controller
     */
    case 4:
      // lastRotateOutput =
      // Robot.drivetrain.rotateDriveStraightController.getControlOutput();
      // headingCorrection =
      // (Robot.drivetrain.rotateDriveStraightController.getControlOutput());
      //
      break;

    default:

    }
    /**
     * Defaults to Tank Drive
     */

    Robot.drivetrain.driveLeft(OI.getInstance().driverJoystick.getLeftStickRaw_Y());
    Robot.drivetrain.driveRight(OI.getInstance().driverJoystick.getRightStickRaw_Y());

  }

  // Called repeatedly when this Command is scheduled to run
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
