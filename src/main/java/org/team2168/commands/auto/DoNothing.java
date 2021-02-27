package org.team2168.commands.auto;


import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Does nothing, absolutely nothing
 */
public class DoNothing extends CommandBase {

	public DoNothing() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	public void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	public void end(boolean interrupted) {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	public void interrupted() {
	}
}
