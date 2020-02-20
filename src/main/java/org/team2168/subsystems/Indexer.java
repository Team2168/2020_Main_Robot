/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.utils.consoleprinter.ConsolePrinter;

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

  public volatile double indexerMotorVoltage;

  private Indexer(){
    _motor = new CANSparkMax(RobotMap.INDEXER_MOTOR_PDP, MotorType.kBrushed);
    entranceLineBreak = new DigitalInput(RobotMap.ENTRANCE_LINE_BREAK);
    exitLineBreak = new DigitalInput(RobotMap.EXIT_LINE_BREAK);
    _motor.setIdleMode(IdleMode.kBrake);

    indexerMotorVoltage = 0;

    _motor.setSmartCurrentLimit(30);
    _motor.setControlFramePeriodMs(20);
    _motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
    _motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    _motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

    ConsolePrinter.putBoolean("Cell Entering", () -> {return this.isBallEntering();}, true, false);
    ConsolePrinter.putBoolean("Cell Exiting", () -> {return this.isBallExiting();}, true, false);
    ConsolePrinter.putNumber("IndexerCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.INDEXER_MOTOR_PDP);}, true, false);
    ConsolePrinter.putNumber("IndexerVoltage", () -> {return this.getIndexerMotorVoltage();}, true, false);
    
  }

  public static Indexer getInstance(){
    if(_instance == null){
      _instance = new Indexer();
    }
    return _instance;
  }  

  /**
    * Cycles the indexer
    * @param speed 1.0 to -1.0,  positive is toward the shooter, negative is away from the shooter
    */
  public void drive(double speed) {
    if(_INDEXER_MOTOR_REVERSED) {
      speed = speed * -1;
    }
    _motor.set(speed);

    indexerMotorVoltage = Robot.pdp.getBatteryVoltage() * speed;
  }

  public boolean isBallEntering() {
    return entranceLineBreak.get();
  }

  public boolean isBallExiting() {
    return exitLineBreak.get();
  }

  public double getIndexerMotorVoltage(){
    return indexerMotorVoltage;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new DriveIndexerWithJoystick());
  }
}
