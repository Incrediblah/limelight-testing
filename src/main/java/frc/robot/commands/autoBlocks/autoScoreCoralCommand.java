// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoBlocks;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.CoralIntakeCmd;
import frc.robot.commands.OdometryCmd;
import frc.robot.commands.limelightCommands.alignXLeftCamera;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoScoreCoralCommand extends SequentialCommandGroup {
  /** Creates a new autoScoreCoralCommand. */

  
  Command visionCommand; 

  public autoScoreCoralCommand(DriveSubsystem drive, VisionSubsystem vision, ElevatorSubsystem elevator, ArmSubsystem arm, CoralIntakeSubsystem intake, String reefside) {  
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Command alignLeft = new alignXLeftCamera(drive, vision, 0, false, 0, 0); 
    Command alignRight = new alignXLeftCamera(drive, vision, 0, false, 0, 0); 

    if(reefside == "left"){
      visionCommand = alignLeft; 
    }else if(reefside == "right"){
      visionCommand = alignRight; 
    }

    addCommands(
      new ParallelCommandGroup(
        visionCommand, 
        new  autoPositionArmUp(arm, elevator, ArmConstants.kLevel4, ElevatorConstants.kLevel4)
      ),

      new CoralIntakeCmd (intake, CoralIntakeConstants.kCoralOutakeSpeed)

    );
  }
}
