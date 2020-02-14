/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team2168.RobotMap;
import org.team2168.commands.intakeMotor.DriveIntakeWithJoystick;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class IntakeMotor extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX intakeMotor;
  public boolean INTAKE_MOTOR_REVERSE = false; //change manually
  public static final double maxSpeed = 60;

  private static IntakeMotor _instance = null;

  /**
   * Default constructors for Intake
   */
  private IntakeMotor() {
    intakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR_PDP);
    //The methods for the voltage and current do not yet exist.
    }

  /**
   * @return an instance of the Intake Subsystem
   */
  public static IntakeMotor getInstance() {
    if (_instance == null)
    {
      _instance = new IntakeMotor();
    }
    return _instance;
  }

    /**
   * sets motor speed to input, postive is towards the robot
   * @param speed -1.0 to 1.0. negative is away from the robot, 0 is stationary, positive is towards the robot
   */
  public void driveMotor(double speed)
  {
    //can set negative speed if a wire is reversed
    if (INTAKE_MOTOR_REVERSE) {
      speed = -speed;
    }
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new DriveIntakeWithJoystick());
  }
}