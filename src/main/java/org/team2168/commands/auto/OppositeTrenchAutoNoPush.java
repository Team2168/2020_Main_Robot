/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.auto;

import org.team2168.commands.auto.robotFunctions.FireBallsAuto;
import org.team2168.commands.drivetrain.PIDCommands.DriveXDistance;
import org.team2168.commands.drivetrain.PIDCommands.TurnXAngle;
import org.team2168.commands.hood_adjust.MoveToWLNoShoot;
import org.team2168.commands.hopper.DriveHopperWithConstant;
import org.team2168.commands.indexer.DriveIndexerWithConstant;
import org.team2168.commands.intakeMotor.DriveIntakeWithConstant;
import org.team2168.commands.intakePivot.ExtendIntakePneumatic;
import org.team2168.commands.intakePivot.RetractIntakePneumatic;
import org.team2168.commands.shooter.DriveToXSpeed;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class OppositeTrenchAutoNoPush extends CommandGroup {
  /**
   * Add your docs here.
   */
  public OppositeTrenchAutoNoPush() {
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
    addParallel(new MoveToWLNoShoot());
    addParallel(new DriveToXSpeed(3250.0));
    
    //drive and intake
    addParallel(new DriveIntakeWithConstant(0.95));//TODO set
    addSequential(new ExtendIntakePneumatic());
    addSequential(new DriveXDistance(-93.0, 0.5)); //91 in working inner port
    addParallel(new DriveIntakeWithConstant(0.3));
    addSequential(new RetractIntakePneumatic()); 

    //turn and drive to firing location 
    addSequential(new TurnXAngle(-63.0, 0.5), 2.0); //68
    addSequential(new DriveXDistance(167.0, 0.5), 4.0);  
    addSequential(new TurnXAngle(45.5, 0.4), 2.0); //45.8--four inner port, not in line for next pickup
    
    //Fire 
    // addSequential(new FireBallsAutoNoLineBreak(), 2.0);
    addSequential(new FireBallsAuto(5), 2.0);
    addParallel(new DriveHopperWithConstant(0.0), 0.1);
    addParallel(new DriveIndexerWithConstant(0.0), 0.0);
    // addSequential(new TurnXAngle(-30.0, 0.3), 2.0); //45.8--four inner port, not in line for next pickup

    // addParallel(new MoveToFrontTrench()); 

    addParallel(new DriveIntakeWithConstant(0.95));//TODO set
    addSequential(new ExtendIntakePneumatic());
    addSequential(new DriveXDistance(-68.0, 0.5));

    addParallel(new DriveIntakeWithConstant(0.3));
    addSequential(new RetractIntakePneumatic());
    
    addSequential(new DriveXDistance(+68.0, 0.5));


    // addSequential(new FireBallsAutoNoLineBreak(), 2.0);
    addSequential(new FireBallsAuto(2), 2.0);


    // addSequential(new TurnXAngle(-18.0, 0.3), 2.0);
    // addSequential(new DriveXDistance(-18.0, 0.5), 2.0);
    // addParallel(new DriveIntakeWithConstant(0.3));
    // addSequential(new RetractIntakePneumatic()); 

    //addSequential(new TurnXAngle(35.0, 0.3));


  }
}
