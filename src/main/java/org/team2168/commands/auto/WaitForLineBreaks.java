/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.auto;

import org.team2168.subsystems.Climber;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeMotor;

import edu.wpi.first.wpilibj.command.Command;

public class WaitForLineBreaks extends Command {
  private double _indexerSpeed;
  private double _hopperSpeed;
  private double _intakeSpeed;
  private int _numBalls;
  private boolean exitLineBreakLast;
  private int ballCounter;
  /**
   * if line break triggers too many times from balls jittering, 
   * add counter that iterates loops that the ball visible to the line break
   * above a certain threshold should be a ball actually going through 
   * and not just jitters
   */
  private Indexer indexer;
  private Climber climber;
  private IntakeMotor intake;
  public WaitForLineBreaks(double indexerSpeed, double hopperSpeed, double intakeSpeed, int numBalls) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    indexer = Indexer.getInstance();
    climber = Climber.getInstance();
    intake = IntakeMotor.getInstance();
    requires(indexer);
    requires(climber);
    requires(intake);

    this._indexerSpeed = indexerSpeed;
    this._hopperSpeed = hopperSpeed;
    this._intakeSpeed = intakeSpeed;
    this._numBalls = numBalls;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    exitLineBreakLast = indexer.isBallExiting();
    ballCounter = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (exitLineBreakLast && !indexer.isBallExiting())
    {
      ballCounter++;
    }

    exitLineBreakLast = indexer.isBallExiting();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return ballCounter < _numBalls;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
