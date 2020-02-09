/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Indexer extends Subsystem {
  // Puts methods for controlling this subsystem
  // here. Call these from Commands.
  private final boolean _INDEXER_MOTOR_REVERSED = false;
  private CANSparkMax _motor;
  private static DigitalInput entranceLineBreak;
  private static DigitalInput exitLineBreak;
  private static Indexer _instance = null;

  private Indexer(){
    _motor = new CANSparkMax(RobotMap.INDEXER_MOTOR_PDP, MotorType.kBrushless);
    entranceLineBreak = new DigitalInput(RobotMap.ENTRANCE_LINE_BREAK);
    exitLineBreak = new DigitalInput(RobotMap.EXIT_LINE_BREAK);
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
    if(_INDEXER_MOTOR_REVERSED) {
      speed = speed * -1;
    }
    _motor.set(speed);
  }

  public boolean isBallEntering() {
    return entranceLineBreak.get();
  }

  public boolean isBallExiting() {
    return exitLineBreak.get();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

  
  }
}
