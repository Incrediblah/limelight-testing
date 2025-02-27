// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoBlocks;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.MoveArmToSetpoint;
import frc.robot.commands.MoveElevatorToSetpoint;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoPositionArmUp extends SequentialCommandGroup {
  /** Creates a new autoPositionArrm. */
  public autoPositionArmUp(ArmSubsystem arm, ElevatorSubsystem elevator, double armSetPoint, double elevatorSetPoint ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
      // new ParallelCommandGroup(
      //   new MoveArmToSetpoint(arm, armSetPoint), 
      //   new MoveElevatorToSetpoint(elevator, elevatorSetPoint)
      // ); 

    addCommands(
      new MoveArmToSetpoint(arm, armSetPoint), 
      new MoveElevatorToSetpoint(elevator, elevatorSetPoint) 
    );


  }
}
