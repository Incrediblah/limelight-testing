// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelightCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class autoAlignY extends Command {

  private VisionSubsystem VISION_SUBSYSTEM; 
  private DriveSubsystem DRIVE_SUBSYSTEM; 

  private PIDController drivePID; 

  private boolean endCommand; 
  private int setPipelineNumber; 

  private double measuredValue; 
  private double driveSpeed; 
  private double tolerance;

  private double targetValue; 

  /** Creates a new autoAlignX. */
  public autoAlignY(DriveSubsystem drive, VisionSubsystem vision, int pipeline, boolean end, double targetOffset,double tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.DRIVE_SUBSYSTEM = drive; 
    this.VISION_SUBSYSTEM = vision; 

    this.drivePID = new PIDController(0.05, 0, 0); 
    this.endCommand = end; 
    this.setPipelineNumber = pipeline; 
    this.targetValue = targetOffset; 
    addRequirements(VISION_SUBSYSTEM);
    addRequirements(DRIVE_SUBSYSTEM);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivePID.reset();
    VISION_SUBSYSTEM.setPipeline(setPipelineNumber);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(VISION_SUBSYSTEM.limelightTargetSeen()){
      measuredValue = VISION_SUBSYSTEM.getTy(); 

      if (Math.abs(Math.abs(measuredValue) - Math.abs(targetValue)) <= tolerance) { 

        driveSpeed = 0; 
      } 
      else{

        driveSpeed = drivePID.calculate(measuredValue,targetValue);
      }

      if(driveSpeed > 0.1){
        driveSpeed = 0.1; 
      }else if(driveSpeed < -0.1){
        driveSpeed = -0.1; 
      }
      
      DRIVE_SUBSYSTEM.drive(-driveSpeed, 0, 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    driveSpeed = 0; 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(endCommand){
      return true; 
    }else{
      return false; 
    }
  }
}
