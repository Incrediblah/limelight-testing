// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoBlocks;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.CoralIntakeCmd;
import frc.robot.commands.OdometryCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoGoToFeederandIntakeCmd extends SequentialCommandGroup {
  /** Creates a new autoGoToFeederStation. */
  public autoGoToFeederandIntakeCmd(DriveSubsystem drive,ArmSubsystem arm, ElevatorSubsystem elevator, CoralIntakeSubsystem intake, String path) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new OdometryCmd(drive, path),
        new autoPositionArmDown(arm, elevator, ArmConstants.kLevel1, ElevatorConstants.kLevel1)
      ),
       new CoralIntakeCmd(intake, CoralIntakeConstants.kCoralIntakeSpeed)
    );
  }
  
}
