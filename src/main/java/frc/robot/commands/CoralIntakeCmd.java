// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntakeCmd extends Command {

  private final CoralIntakeSubsystem s_CoralIntakeSubsystem;
  private final double CoralIntakeSpeed;

  /** Creates a new CoralIntakeCmd. */
  public CoralIntakeCmd(CoralIntakeSubsystem s_CoralIntakeSubsystem, double CoralIntakeSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_CoralIntakeSubsystem = s_CoralIntakeSubsystem;
    this.CoralIntakeSpeed = CoralIntakeSpeed;
    addRequirements(s_CoralIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //s_CoralIntakeSubsystem.setCoralIntakeVelocity(CoralIntakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_CoralIntakeSubsystem.setCoralIntakeVelocity(CoralIntakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //s_CoralIntakeSubsystem.stopCoralIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
