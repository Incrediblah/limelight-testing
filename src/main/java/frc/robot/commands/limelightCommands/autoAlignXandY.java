// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelightCommands;

import org.opencv.features2d.FastFeatureDetector;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class autoAlignXandY extends Command {

  private VisionSubsystem VISION_SUBSYSTEM; 
  private DriveSubsystem DRIVE_SUBSYSTEM; 

  private PIDController strafePIDx; 
  private PIDController strafePIDy; 

  private boolean endCommand; 
  private int setPipelineNumber; 

  private double measuredValuex; 
  private double measuredValuey; 
  private double strafeSpeedx; 
  private double driveSpeedy; 

  private double targetValuex; 
  private double targetValuey; 

  private double xTolreance; 
  private double yTolerance; 

  /** Creates a new autoAlignX. */
  public autoAlignXandY(DriveSubsystem drive, VisionSubsystem vision, int pipeline, boolean end, double targetOffsetx, double targetOffsety,double tolerancex,double tolerancey) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.DRIVE_SUBSYSTEM = drive; 
    this.VISION_SUBSYSTEM = vision; 

    this.strafePIDx = new PIDController(0.015, 0, 0); 
    this.strafePIDy = new PIDController(0.075, 0, 0); 
    this.endCommand = end; 
    this.setPipelineNumber = pipeline; 
    this.targetValuex = targetOffsetx; 
    this.targetValuey = targetOffsety; 
    this.xTolreance = tolerancex; 
    this.yTolerance = tolerancey; 
    addRequirements(VISION_SUBSYSTEM);
    addRequirements(DRIVE_SUBSYSTEM);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    strafePIDx.reset();
    strafePIDy.reset();
    VISION_SUBSYSTEM.setPipeline(setPipelineNumber);
    DRIVE_SUBSYSTEM.resetOdometry(DRIVE_SUBSYSTEM.getPose());
    endCommand = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(VISION_SUBSYSTEM.limelightTargetSeen()){
      measuredValuey = VISION_SUBSYSTEM.getTy(); 
      measuredValuex = VISION_SUBSYSTEM.getTx(); 

      if (Math.abs(Math.abs(measuredValuex) - Math.abs(targetValuex)) <= xTolreance && (Math.abs(measuredValuey) - Math.abs(targetValuey)) <= yTolerance) { 
        strafeSpeedx = 0; 
        driveSpeedy = 0; 
        endCommand = true; 
      } 

      else{
      strafeSpeedx = strafePIDx.calculate(measuredValuex,targetValuex);
      driveSpeedy = strafePIDy.calculate(measuredValuey,targetValuey);
      }
     

      if(strafeSpeedx > 0.25){
        strafeSpeedx = 0.25; 
      }else if(strafeSpeedx < -0.25){
        strafeSpeedx = -0.25; 
      }

      
      if(driveSpeedy>0.25){ 
        driveSpeedy = 0.25; 
      }else if(driveSpeedy<-0.25){
        driveSpeedy = -0.25; 
      }
      
    }else{
      strafeSpeedx = 0; 
      driveSpeedy = 0; 
    }

    // SmartDashboard.putNumber("align speed", strafeSpeedx); 
    DRIVE_SUBSYSTEM.drive(-driveSpeedy, strafeSpeedx, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    strafeSpeedx = 0; 
    driveSpeedy = 0; 
    DRIVE_SUBSYSTEM.resetOdometry(DRIVE_SUBSYSTEM.getPose());
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
