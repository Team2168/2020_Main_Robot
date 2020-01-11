/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private VictorSP intakeMotor;
  private DoubleSolenoid intakePivot;

  private static Intake _instance = null;

  private Intake() {
    intakeMotor = new VictorSP(RobotMap.INTAKE_MOTOR_PDP);
    intakePivot = new DoubleSolenoid(RobotMap.INTAKE_ENGAGE_PCM, RobotMap.INTAKE_DISENGAGE_PCM)
  }

  public static Intake getInstance() {
    if (_instance == null)
    {
      _instance = new Intake();
    }
    return _instance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void extendIntake() {
    intakePivot.set(DoubleSolenoid.Value.kforward);
  }

  public void retractIntake() {
    intakePivot.set(DoubleSolenoid.Value.kreverse);
  }

}
