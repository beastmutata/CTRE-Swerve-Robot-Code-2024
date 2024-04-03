package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class ClimberSubsystem extends SubsystemBase {

  private CANSparkMax LeftClimber;
  private CANSparkMax RightClimber;

  /** Creates a new IntakeSubsystem. */
  public ClimberSubsystem() {

    LeftClimber = new CANSparkMax(IntakeConstants.LeftClimberID, MotorType.kBrushed);
    RightClimber = new CANSparkMax(IntakeConstants.RightCLimberID, MotorType.kBrushed);

    // pivotMotorConfig

    LeftClimber.restoreFactoryDefaults();
    RightClimber.restoreFactoryDefaults();

  }
  @Override
  public void periodic() {
/////idk but ima not put something here
  }


  public void setLeftSpeed(double speed) {
    LeftClimber.set(speed);
  }

  public void setRightSpeed(double speed) {
    RightClimber.set(speed);
  }
}
