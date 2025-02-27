// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelightCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class autoAlignX extends Command {

  private VisionSubsystem VISION_SUBSYSTEM; 
  private DriveSubsystem DRIVE_SUBSYSTEM; 

  private PIDController strafePID; 

  private boolean endCommand; 
  private int setPipelineNumber; 

  private double measuredValue; 
  private double strafeSpeed; 
  private double tolerance;

  private int targetValue; 

  /** Creates a new autoAlignX. */
  public autoAlignX(DriveSubsystem drive, VisionSubsystem vision, int pipeline, boolean end, int targetOffset,int tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.DRIVE_SUBSYSTEM = drive; 
    this.VISION_SUBSYSTEM = vision; 

    this.strafePID = new PIDController(0.01, 0, 0); 
    this.endCommand = end; 
    this.setPipelineNumber = pipeline; 
    this.targetValue = targetOffset; 
    addRequirements(VISION_SUBSYSTEM);
    addRequirements(DRIVE_SUBSYSTEM);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    strafePID.reset();
    VISION_SUBSYSTEM.setPipeline(setPipelineNumber);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(VISION_SUBSYSTEM.limelightTargetSeen()){
      measuredValue = VISION_SUBSYSTEM.getTx(); 

      if (Math.abs(Math.abs(measuredValue) - Math.abs(targetValue)) <= tolerance) { 

        strafeSpeed = 0; 
      } 
      else{

      strafeSpeed = strafePID.calculate(measuredValue,targetValue);
      }
     

      if(strafeSpeed > 0.1){
        strafeSpeed = 0.1; 
      }else if(strafeSpeed < -0.1){
        strafeSpeed = -0.1; 
      }
      
    }else{
      strafeSpeed = 0; 
    }

    SmartDashboard.putNumber("align speed", strafeSpeed); 
    DRIVE_SUBSYSTEM.drive(0, strafeSpeed, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    strafeSpeed = 0; 
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
