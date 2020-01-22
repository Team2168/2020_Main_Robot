/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/* the purpose of this class is to control the intake motor without the joystick*/

package org.team2168.commands.intake;

import org.team2168.subsystems.Intake;

import edu.wpi.first.wpilibj.command.Command;

public class DriveWithConstant extends Command {
  double speed;
  private Intake intake = Intake.getInstance();  

  public DriveWithConstant(double inputSpeed) { //when the command is initialized the speed is put straight in
    requires(intake);
    this.speed = inputSpeed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  /**
   * sets motor speed to the input
   * @author Ian
   */
  @Override
  protected void execute() {
    intake.driveMotor(speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    intake.driveMotor(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}