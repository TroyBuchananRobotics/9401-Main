package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Elevator extends SubsystemBase {
    private final CANSparkMax m_motor = new CANSparkMax(9, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkMaxPIDController m_PID = m_motor.getPIDController();
    private final ElevatorFeedforward m_ff = new ElevatorFeedforward(0.0, 0.25, 0);

    private double m_position = 5;

    public Elevator(){
        m_motor.setSmartCurrentLimit(CurrentLimit.kElevator);
        m_motor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float)2.0);
        m_motor.setSoftLimit(SoftLimitDirection.kForward, (float)82.5);
        m_motor.setOpenLoopRampRate(0.10);
        m_motor.setClosedLoopRampRate(0.10);
        m_motor.setInverted(true);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_encoder.setPositionConversionFactor(1.0);
        m_encoder.setVelocityConversionFactor(1.0);
        m_PID.setP(0.1,0); //0.3

        m_motor.burnFlash();
    }

    public void setPosition(double position){
        m_position = position;
    }

    public double getPosition(){
        return m_encoder.getPosition();
    }

    public double getPositionError(){
        return m_position-getPosition();
    }

    @Override
    public void periodic(){
        m_PID.setReference(m_position, ControlType.kPosition,0,0.0);//m_ff.calculate(0.0));
        SmartDashboard.putNumber("Elevator Setpoint", m_position);
        SmartDashboard.putNumber("Elevator Pose", getPosition());
        SmartDashboard.putNumber("Elevator Output", m_motor.getAppliedOutput());
    }


}
