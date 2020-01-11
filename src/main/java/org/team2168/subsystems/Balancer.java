//This is using PWM, falcon 500

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.team2168.RobotMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;


/**
 * Add your docs here.
 */
public class Balancer extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSolenoid Brakes;
  private Brakes();
  { 
    Brakes = new DoubleSolenoid(RobotMap.BALANCER_ENGAGE_PCM, RobotMap.BALANCER_DISENGAGE_PCM);
  }
  
  public void engageBrakes();
  {
    Brakes.set(DoubleSolenoid.Value.kForward);
  }

  public void disengageBreaks();
  {
    Brakes.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean areBrakesEngaged();
  {
    return _Brakes.get() == DoubleSolenoid.Value.kforward;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
