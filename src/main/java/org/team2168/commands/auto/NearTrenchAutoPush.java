/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.auto;

import org.team2168.Robot;
import org.team2168.commands.auto.robotFunctions.FireBallsAuto;
import org.team2168.commands.auto.robotFunctions.FireBallsAutoNoLineBreak;
import org.team2168.commands.drivetrain.PIDCommands.DriveXDistance;
import org.team2168.commands.drivetrain.PIDCommands.TurnXAngle;
import org.team2168.commands.hood_adjust.MoveToFrontTrench;
import org.team2168.commands.hood_adjust.MoveToWallNoShoot;
import org.team2168.commands.hopper.DriveHopperWithConstant;
import org.team2168.commands.indexer.DriveIndexerWithConstant;
import org.team2168.commands.intakeMotor.DriveIntakeWithConstant;
import org.team2168.commands.intakePivot.ExtendIntakePneumatic;
import org.team2168.commands.intakePivot.RetractIntakePneumatic;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class NearTrenchAutoPush extends CommandGroup {
  /**
   * Add your docs here.
   */
  public NearTrenchAutoPush() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    addParallel(new MoveToFrontTrench());
    addParallel(new DriveIntakeWithConstant(1.0));
    addParallel(new ExtendIntakePneumatic()); //switch to parallel because this can happen while we push

    //push robot off line
    addSequential(new DriveXDistance(4.0, 0.5, 10.0*120.0, 1));

    addSequential(new DriveXDistance(-115.0 - 5.0, 0.5), 4.0); //TODO VERIFY MEASUREMENTS
    addParallel(new DriveIntakeWithConstant(0.3));
    addSequential(new RetractIntakePneumatic());  //want to run the intake constantly during firing... TODO: cleanup
    addSequential(new TurnXAngle(12.25, 0.4), 2.0); //12.25), 2.0);
    if(Robot.isPracticeBot()) {
    addSequential(new FireBallsAutoNoLineBreak(), 2.0);
    }
    else {
      addSequential(new FireBallsAuto(5), 2.0);
    }
    addParallel(new MoveToWallNoShoot());

    // //turn straight again
    addSequential(new TurnXAngle(-12.0, 0.4), 2.0);
    addParallel(new DriveIntakeWithConstant(1.0));//TODO set
    addSequential(new ExtendIntakePneumatic());
    addSequential(new DriveXDistance(-103.0, 0.5), 4.0);
    addSequential(new DriveXDistance(103.0, 0.5), 4.0);
  
    addParallel(new DriveIntakeWithConstant(0.3));
    addSequential(new RetractIntakePneumatic()); 

    addParallel(new MoveToFrontTrench());
    addSequential(new TurnXAngle(+12.0, 0.4), 2.0);
    if(Robot.isPracticeBot()) {
      addSequential(new FireBallsAutoNoLineBreak(), 4.0);
      }
      else {
        addSequential(new FireBallsAuto(3), 2.0);
      }
    addParallel(new DriveIndexerWithConstant(0.0), 0.0);
    addParallel(new DriveHopperWithConstant(0.0), 0.0);
    addParallel(new DriveIntakeWithConstant(0.0), 0.0);
  }
}
