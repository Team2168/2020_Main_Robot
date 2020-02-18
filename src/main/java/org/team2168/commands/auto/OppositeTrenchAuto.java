/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.auto;

import org.team2168.commands.drivetrain.PIDCommands.DriveXDistance;
import org.team2168.commands.drivetrain.PIDCommands.TurnXAngle;
import org.team2168.commands.hood_adjust.MoveToFrontTrench;
import org.team2168.commands.hood_adjust.MoveToWhiteLine;
import org.team2168.commands.hopper.DriveHopperWithConstant;
import org.team2168.commands.indexer.DriveIndexerWithConstant;
import org.team2168.commands.intakeMotor.DriveIntakeWithConstant;
import org.team2168.commands.intakePivot.ExtendIntakePneumatic;
import org.team2168.commands.intakePivot.RetractIntakePneumatic;
import org.team2168.commands.shooter.DriveToXSpeed;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class OppositeTrenchAuto extends CommandGroup {
  /**
   * Add your docs here.
   */
  public OppositeTrenchAuto() {
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

    //start shooter
    addParallel(new MoveToWhiteLine());
    addParallel(new DriveToXSpeed(3300.0)); //blind guess
    
    //drive and intake
    addParallel(new DriveIntakeWithConstant(1.0));//TODO set
    addSequential(new ExtendIntakePneumatic());
    addSequential(new DriveXDistance(-91.0, 0.5)); //91 in working inner port
    addParallel(new DriveIntakeWithConstant(0.3));
    addSequential(new RetractIntakePneumatic()); 

    //turn and drive to firing location 
    addSequential(new TurnXAngle(-60.0, 0.5), 2.0);
    addSequential(new DriveXDistance(180.0, 0.5), 4.0); //176, 
    addSequential(new TurnXAngle(47.8, 0.3), 2.0); //45.8--four inner port, not in line for next pickup
    
    //Fire 
    addSequential(new FireBallsAutoNoLineBreak(), 2.5);
    addParallel(new DriveHopperWithConstant(0.0), 0.1);
    addParallel(new DriveIndexerWithConstant(0.0), 0.0);

    addParallel(new MoveToFrontTrench());

    addParallel(new DriveIntakeWithConstant(0.95));//TODO set
    addSequential(new ExtendIntakePneumatic());
    addSequential(new DriveXDistance(-76.0, 0.5));

    addSequential(new TurnXAngle(-18.0, 0.3), 2.0);
    addSequential(new DriveXDistance(-18.0, 0.5), 2.0);
    addParallel(new DriveIntakeWithConstant(0.3));
    addSequential(new RetractIntakePneumatic()); 

    //addSequential(new TurnXAngle(35.0, 0.3));


  }
}
