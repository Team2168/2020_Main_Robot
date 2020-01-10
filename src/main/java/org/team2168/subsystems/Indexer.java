/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. EVERY SINGLE Right Big or Small is Reserved.     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import java.lang.reflect.Constructor;

import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Indexer extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSolenoid Piston; 
  private static Indexer instance = null;
  private Indexer(){
    Piston = new DoubleSolenoid(RobotMap.Indexer_Engage_PCM, RobotMap.Indexer_Disengage_PCM)
  }
 public static Indexer GetInstance(){
   if(instance == null){
     instance = new Indexer();
   }
   return instance;
   
 }  
public void extendPiston(){
  Piston.set(DoubleSolenoid.Value.kForward);
  }

public void retractPiston(){
  Piston.set(DoubleSolenoid.Value.kReverse);
  }

public boolean isExtended(){
  return Piston.get() == DoubleSolenoid.Value.kForward;
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

  
  }
}
