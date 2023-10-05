package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ZeroRoutine extends CommandBase{
    private final Claw m_claw;
    private boolean m_closed;
    private double currentDraw;
    private double position;
    private final PIDController m_PID = new PIDController(0.025, 0, 0);

    public ZeroRoutine(Claw claw){
        m_claw = claw;
        addRequirements(m_claw);
    }

    @Override
    public void initialize(){
        m_closed = false;

    }

    @Override
    public void execute(){
        currentDraw = m_claw.getAppliedCurrent();
        position = m_claw.getPose();

        
    }
}
