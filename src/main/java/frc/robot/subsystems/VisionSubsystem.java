// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  private final NetworkTable m_limelightTable; 
  private final String VISION_PREFIX = "Vision/"; 

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight"); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean(VISION_PREFIX + "LIMELIGHT TARGET SEEN", limelightTargetSeen()); 
    SmartDashboard.putNumber(VISION_PREFIX + "LIMELIGHT TX", getTx());
    SmartDashboard.putNumber(VISION_PREFIX + "LIMELIGHT TY", getTy());
    SmartDashboard.putNumber(VISION_PREFIX + "LIMELIGHT TA", getTa());
    SmartDashboard.putNumber(VISION_PREFIX + "LIMELIGHT TV", getTv());
  }
 
  

  public NetworkTable limelightTableRead(){
    return m_limelightTable; 
  }

  public double getTx(){
    return m_limelightTable.getEntry("tx").getDouble(0); 
  }

  public double getTy(){
    return m_limelightTable.getEntry("ty").getDouble(0); 
  }

  
  public double getTa(){
    return m_limelightTable.getEntry("ta").getDouble(0); 
  }

  public double getTv(){
    return m_limelightTable.getEntry("tv").getDouble(0); 
  }

  public double specificTx(int index){
    return m_limelightTable.getEntry("tx" + index).getDouble(0); 
  }

  public void setPipeline(int pipelineNumber){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineNumber); 
  }

  public void setLED(int ledMode){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledMode); 
  }

  public int numberOfTargets(){
    double tvValues = m_limelightTable.getEntry("tv").getDouble(0);
    int tvVal = (int)tvValues; 
    return tvVal; 
  }

  public double targetArea(int index){
    return m_limelightTable.getEntry("ta" + index).getDouble(0); 
  }

  public boolean limelightTargetSeen(){
    if(numberOfTargets() > 0){
      return true; 
    }

    else{
      return false;
    }
  }

}
