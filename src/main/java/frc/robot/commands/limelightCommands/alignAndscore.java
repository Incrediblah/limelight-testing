// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.limelightCommands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.CoralIntakeConstants;
// import frc.robot.Constants.ElevatorConstants;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.commands.limelightCommands.autoAlignXandY;
// import frc.robot.commands.CoralIntakeCmd;
// import frc.robot.commands.MoveArmToSetpoint;
// import frc.robot.commands.MoveElevatorToSetpoint;
// import frc.robot.commands.ReadyUpCmd;

// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
// import frc.robot.subsystems.CoralIntakeSubsystem;



// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class alignAndscore extends SequentialCommandGroup {
//   /** Creates a new autoHuntNoteCommand. */
//   public alignAndscore(DriveSubsystem drive, VisionSubsystem crodie, CoralIntakeSubsystem intake) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//       //you gonna have to tweak these values a lil
//       //have the setpoints be a constants from newest files on github
//       //chage entire constaants folder to the one on github
//       new SequentialCommandGroup(
//         new autoAlignXandY(drive, crodie, 0, isScheduled(), 0, 0, 0),
//         new ReadyUpCmd(null, 0, null, 0),
//         new CoralIntakeCmd(intake, 0)
        
        
//        // new LimeLightAllignCmd(crodie, drive)
        
      
//       )
//     );
//   }}
