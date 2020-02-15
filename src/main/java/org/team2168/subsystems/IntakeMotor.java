/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class IntakeMotor extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX intakeMotor;
  public boolean INTAKE_MOTOR_REVERSE = true; //change manually
  public static final double MAX_SPEED = 0.8;

  private static IntakeMotor _instance = null;
  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 20; //amps
  private final double TRIGGER_THRESHOLD_LIMIT = 30; //amp
  private final double TRIGGER_THRESHOLD_TIME = 200; //ms

  /**
   * Default constructors for Intake
   */
  private IntakeMotor() {
    intakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR_PDP);

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
    CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);
    intakeMotor.configSupplyCurrentLimit(talonCurrentLimit);

    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setInverted(INTAKE_MOTOR_REVERSE);
    intakeMotor.configNeutralDeadband(0.05);
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
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveIntakeWithJoystick());
  }
}