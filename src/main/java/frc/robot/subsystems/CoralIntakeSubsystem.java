// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralIntakeConstants;

public class CoralIntakeSubsystem extends SubsystemBase {

  private SparkFlex m_CoralIntake = new SparkFlex(CoralIntakeConstants.kCoralIntakeCanId, MotorType.kBrushless);
  private SparkClosedLoopController m_CoralIntakeController = m_CoralIntake.getClosedLoopController();
  private RelativeEncoder m_CoralIntakeEncdoer = m_CoralIntake.getEncoder();

  /** Creates a new CoralIntakeSubsystem. */
  public CoralIntakeSubsystem() {

    m_CoralIntake.configure(
      Configs.CoralIntakeSubsystem.coralIntakeConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );

    m_CoralIntakeEncdoer.setPosition(0);

  }

  public void setCoralIntakeVelocity(double CoralIntakeSpeed) {
    m_CoralIntake.set(CoralIntakeSpeed);
    //m_CoralIntakeController.setReference(CoralIntakeSpeed, ControlType.kMAXMotionVelocityControl);
  }

  public void stopCoralIntake() {
    m_CoralIntake.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
