package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Claw extends SubsystemBase{
    private final CANSparkMax m_motor = new CANSparkMax(11, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkMaxPIDController m_PID = m_motor.getPIDController();

    private double m_velocity = 0.0;
    public Claw(){
        m_motor.setSmartCurrentLimit(CurrentLimit.kClaw);
        m_motor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor.setInverted(false);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_encoder.setPositionConversionFactor(1.0);
        m_encoder.setVelocityConversionFactor(1.0);
        m_PID.setFF(0.000175);
        m_PID.setP(0.00015);
        m_motor.burnFlash();
    }

    public void setVelocity(double velocity){
        m_velocity = velocity;
    }

    public void zero(){
        m_encoder.setPosition(0.0);
    }

    public double getVelocity(){
        return m_encoder.getVelocity();
    }

    public double getAppliedCurrent(){
        return m_motor.getAppliedOutput();
    }

    public double getPose(){
        return m_encoder.getPosition();
    }

    @Override
    public void periodic(){
        m_PID.setReference(m_velocity, ControlType.kVelocity);
        SmartDashboard.putNumber("Claw Speed",getVelocity());
    }
}
