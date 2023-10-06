package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;

public class ZeroRoutine extends CommandBase{
    private final Wrist m_wrist;
    private boolean m_finished = false;
    private final Timer m_timer = new Timer();

    public ZeroRoutine(Wrist wrist){
        m_wrist = wrist;
        addRequirements(m_wrist);
    }

    @Override
    public void initialize(){
        m_wrist.disablePID();
        m_wrist.zero();
        m_finished = false;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute(){
        if(m_wrist.getCurrent()>17.5 && m_timer.get() > 0.25){
            m_wrist.setZero();
            m_wrist.setPosition(2.0);
            m_wrist.enablePID();
            m_finished = true;
        }
        
    }

    @Override
    public void end(boolean interuppted){
        m_wrist.setPosition(5.0);
        m_wrist.enablePID();
    }

    @Override
    public boolean isFinished(){
        return m_finished;
    }
}
