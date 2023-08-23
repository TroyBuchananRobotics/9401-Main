package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;



public class Wrist extends SubsystemBase{
    private final CANSparkMax m_motor = new CANSparkMax(10, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkMaxPIDController m_PID = m_motor.getPIDController();
    private final ArmFeedforward m_ff = new ArmFeedforward(0.15,-2.0,0.0);

    private final static double m_offset = -43.0;
    private final static double m_conversion = 0.059573905;

    private double m_position = 2;
    
    public Wrist(){
        m_motor.setSmartCurrentLimit(CurrentLimit.kWrist);
        m_motor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float)1.0 );
        m_motor.setSoftLimit(SoftLimitDirection.kForward, (float)60.0 );
        m_motor.setInverted(true);
        m_motor.setOpenLoopRampRate(0.10);
        m_motor.setClosedLoopRampRate(0.10);
        m_motor.setIdleMode(IdleMode.kCoast);
        m_encoder.setPositionConversionFactor(1.0);
        m_encoder.setVelocityConversionFactor(1.0);
        m_PID.setP(0.25);

        m_motor.burnFlash();
    }

    public void setPosition(double poisition){
        m_position = poisition;
    }

    public double getPosition(){
        return m_encoder.getPosition();
    }

    @Override
    public void periodic(){
        double ff = m_ff.calculate(m_conversion*(getPosition()+m_offset), 0.0);
        m_PID.setReference(m_position,ControlType.kPosition,0,ff);
        SmartDashboard.putNumber("Wrist Pose",getPosition());

    }
}
