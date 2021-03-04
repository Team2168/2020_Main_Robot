/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.auto.robotFunctions;


import org.team2168.subsystems.Indexer;



import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitForLineBreaks extends CommandBase {
  private double _indexerSpeed;
  private int _numBalls;
  private boolean exitLineBreakLast;
  private int ballCounter;

  private Indexer indexer;
  public WaitForLineBreaks(double indexerSpeed, int numBalls) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    indexer = Indexer.getInstance();
    addRequirements(indexer);
    this._indexerSpeed = indexerSpeed;

    this._numBalls = numBalls;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    exitLineBreakLast = false;
    ballCounter = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    indexer.drive(_indexerSpeed);
    if (exitLineBreakLast && !indexer.isBallExiting())
    {
      ballCounter++;
    }
    System.out.println(exitLineBreakLast + " " + !indexer.isBallExiting());

    exitLineBreakLast = indexer.isBallExiting();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return ballCounter >= _numBalls;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}
