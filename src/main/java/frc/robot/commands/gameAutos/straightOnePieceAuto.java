// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gameAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.OdometryCmd;
import frc.robot.commands.autoBlocks.autoPositionArmDown;
import frc.robot.commands.autoBlocks.autoScoreCoralCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class straightOnePieceAuto extends SequentialCommandGroup {
  /** Creates a new straightTwoPieceAuto. */
  public straightOnePieceAuto(DriveSubsystem drive,ArmSubsystem arm, ElevatorSubsystem elevator, VisionSubsystem vision, CoralIntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new OdometryCmd(drive, "straight"), 
      new autoScoreCoralCommand(drive, vision, elevator, arm, intake), 
      new OdometryCmd(drive, "straight-back"), 
      new autoPositionArmDown(arm, elevator, ArmConstants.kLevel1, ArmConstants.kLevel1)
    );
  }
}
