/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem class for the Hopper
 */
public class Hopper extends Subsystem {
  private CANSparkMax hopperMotor;

  private static Hopper _instance = null;

  private Hopper() {
    hopperMotor = new CANSparkMax(RobotMap.HOPPER_MOTOR_PDP, MotorType.kBrushless);

    hopperMotor.setSmartCurrentLimit(60);

    hopperMotor.setControlFramePeriodMs(20);
    
    hopperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
    hopperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    hopperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
