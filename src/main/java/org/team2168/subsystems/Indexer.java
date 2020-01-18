/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. EVERY SINGLE Right Big or Small is Reserved.     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Indexer extends Subsystem {
  // Puts methods for controlling this subsystem
  // here. Call these from Commands.
  private final boolean _isReversed = false;
  private CANSparkMax _motor; 
  private static Indexer _instance = null;
  private Indexer(){
    _motor = new CANSparkMax(RobotMap.INDEXER_MOTOR_PDP, MotorType.kBrushless) 
    }
 public static Indexer getInstance(){
   if(_instance == null){
     _instance = new Indexer();
   }
   return _instance;
 }  

 /**
  * Cycles the indexer 
  * - positive is toward the shooter
  * - negative is away from the shooter
  * @param speed is a double to set the speed
  */
public void drive(double speed) {
  if(_isReversed){
    speed = speed * -1;
  }

  _motor.set(speed);
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

  
  }
}
