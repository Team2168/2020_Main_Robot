/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.Robot;
import org.team2168.PID.sensors.LimelightSensor;
import org.team2168.commands.limelight.UpdatePipeline;
import org.team2168.subsystems.Shooter.FiringLocation;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 * Add your docs here.
 */
public class Limelight extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static LimelightSensor limelight;
  private static Limelight _instance = null;

  private static final int PIPELINE_FORWARD_BLUE = 0;
  private static final int PIPELINE_FORWARD_RED = 2;
  private static final int PIPELINE_BACK_TRENCH_BLUE = 1;
  private static final int PIPELINE_BACK_TRENCH_RED = 3;
  private static final int PIPELINE_DRIVER_VIEW = 4;



  private boolean isLimelightEnabled;

  private Limelight() {
    //set up limelight
    limelight = new LimelightSensor();
    limelight.setCamMode(0);
    limelight.setPipeline(PIPELINE_DRIVER_VIEW);
    isLimelightEnabled = false;
  }

  /**
   * @return an instance of the Intake Subsystem
   */
  public static Limelight getInstance() {
    if (_instance == null) {
      _instance = new Limelight();
    }
    return _instance;
  }

  public double getPosition() {
    return limelight.getPos();
  }

  public void enableLimelight(FiringLocation firingLocation) {
    limelight.setCamMode(0);
    limelight.setLedMode(0);
    // if(Robot.driverstation.isFMSAttached())
    // {
      // if(Robot.onBlueAlliance())
      // {
      //   limelight.setPipeline(3);
      // }
      // else
      // {
      //   limelight.setPipeline(2);
      // }
      switch (firingLocation) {
        case WALL : 
          if(Robot.onBlueAlliance())
          {
            limelight.setPipeline(PIPELINE_FORWARD_BLUE); //TODO deal with this--we can't see from wall??
          }
          else
          {
            limelight.setPipeline(PIPELINE_FORWARD_RED);
          }
          break;
        case WHITE_LINE :
          if(Robot.onBlueAlliance())
          {
            limelight.setPipeline(PIPELINE_FORWARD_BLUE);
          }
          else
          {
            limelight.setPipeline(PIPELINE_FORWARD_RED);
          }
          break;
        case FRONT_TRENCH : 
          if(Robot.onBlueAlliance())
          {
            limelight.setPipeline(PIPELINE_FORWARD_BLUE);
          }
          else
          {
            limelight.setPipeline(PIPELINE_FORWARD_RED);
          }
          break;
        case BACK_TRENCH: 
          if(Robot.onBlueAlliance())
          {
            limelight.setPipeline(PIPELINE_BACK_TRENCH_BLUE);
          }
          else
          {
            limelight.setPipeline(PIPELINE_BACK_TRENCH_RED);
          }
          break;
      }
    // }
    // else
    // {
    //   limelight.setPipeline(0);
    // }
    isLimelightEnabled = true;
  }

  public void pauseLimelight()
  {
    limelight.setCamMode(1);
    limelight.setLedMode(1);
    limelight.setPipeline(PIPELINE_DRIVER_VIEW);
    isLimelightEnabled = false;

  }

  public boolean isLimelightEnabled() {
    return isLimelightEnabled;
  }


  /**
   * Sets the LED mode
   * @param ledNumber is an int from 0 to 3
   *                  0 - use the LED Mode set in the current pipeline
   *                  1 - force off
   *                  2 - force blink
   *                  3 - force on
   */
  public void setLedMode(int ledNumber) {
    limelight.setLedMode(ledNumber);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new UpdatePipeline());
  }
}
