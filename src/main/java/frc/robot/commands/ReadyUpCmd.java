// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.MoveElevatorToSetpoint;
import frc.robot.commands.MoveArmToSetpoint;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReadyUpCmd extends SequentialCommandGroup {
  /** Creates a new autoHuntNoteCommand. */
  public ReadyUpCmd(ElevatorSubsystem elevator,double Setpoint, ArmSubsystem arm,double setpoint) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SequentialCommandGroup(
        new MoveArmToSetpoint(arm, setpoint),
        new MoveElevatorToSetpoint(elevator, Setpoint)
       
        
      
      )
    );
  }}
