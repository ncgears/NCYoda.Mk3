
package frc.team1918.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private WPI_TalonSRX feed1; // first stage feed controller
  private WPI_TalonSRX feed2; // second stage feed controller
  private CANSparkMax shoot; // shooter controller
  private double m_shooter_rpm = 0.0; // Current shooter speed
  private double m_shooter_oldrpm = 0.0; // Old shooter speed
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  private Solenoid m_hood;
  /**
   * Creates a new ExampleSubsystem.
   */
  public ShooterSubsystem() {
    //Reset the talonsrx controllers to avoid any leftovers in flash and configure them as desired
    feed1 = new WPI_TalonSRX(Constants.Shooter.SHOOTER_FEED1_MC_ID);
    feed2 = new WPI_TalonSRX(Constants.Shooter.SHOOTER_FEED2_MC_ID);
    feed1.configFactoryDefault();
    feed2.configFactoryDefault();
    feed1.set(ControlMode.PercentOutput, 0);
    feed2.set(ControlMode.PercentOutput, 0);
    feed1.setNeutralMode(NeutralMode.Coast);
    feed2.setNeutralMode(NeutralMode.Coast);
    feed1.setInverted(Constants.Shooter.SHOOTER_FEED1_INVERT);
    feed2.setInverted(Constants.Shooter.SHOOTER_FEED2_INVERT);
    //Setup the SparkMAX controller as desired
    shoot = new CANSparkMax(Constants.Shooter.SHOOTER_SHOOT_MC_ID, MotorType.kBrushless);
    shoot.restoreFactoryDefaults();
    m_pidController = shoot.getPIDController();
    m_encoder = shoot.getEncoder();
    //Setup the solenoid
    m_hood = new Solenoid(Constants.Air.AIR_HOOD_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_shooter_rpm != m_shooter_oldrpm) {
      m_pidController.setReference(m_shooter_rpm, ControlType.kVelocity);
      SmartDashboard.putNumber("ShootSpeed",m_encoder.getVelocity());
      m_shooter_oldrpm=m_shooter_rpm;
    }
  }

  public void setShooterSpeed(double RPM) {
    m_shooter_rpm = Math.min(RPM, Constants.Shooter.SHOOTER_MAX_RPM);
  }

  public void increaseShooterSpeed() {
    m_shooter_rpm = Math.min(m_shooter_rpm + Constants.Shooter.SHOOTER_SPEED_INCREMENT, Constants.Shooter.SHOOTER_MAX_RPM);
  }

  public void decreaseShooterSpeed() {
    m_shooter_rpm = Math.max(m_shooter_rpm - Constants.Shooter.SHOOTER_SPEED_INCREMENT, Constants.Shooter.SHOOTER_MIN_RPM);
  }

  public void setShooterSpeedFromDashboard() {
    double speed = SmartDashboard.getNumber("Shooter Target RPM", 0);
    setShooterSpeed(speed);
  }

  public void updateDashboardShooterSpeed() {
    SmartDashboard.putNumber("Shooter Target RPM",m_shooter_rpm);
  }

  public void raiseHood(boolean up) {
    //send command to air to put hood up or down based on boolean
    m_hood.set(up);
  }

  public void runFeeder(boolean run) {
    //run the feeder based on boolean
    if (run) {
      feed1.set(ControlMode.PercentOutput, Constants.Shooter.SHOOTER_FEED1_SPEED);
      feed2.set(ControlMode.PercentOutput, Constants.Shooter.SHOOTER_FEED2_SPEED);
    } else {
      feed1.set(ControlMode.PercentOutput, 0);
      feed2.set(ControlMode.PercentOutput, 0);
    }
  }

}
