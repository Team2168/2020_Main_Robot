// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.RobotContainer;
import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveWithConstant extends CommandBase {
  /** Creates a new DriveWithJoysticks. */

  Drivetrain dt;
  RobotContainer rc;
  Double leftSpeed;
  Double rightSpeed;
  public DriveWithConstant(Double leftSpeed, Double rightSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    dt = Drivetrain.getInstance();
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;

    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dt.tankDriveVolts(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
