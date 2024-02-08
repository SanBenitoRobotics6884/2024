package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommands extends Command {
    private ClimbSubsystem m_climbSubsystem;
    private Joystick m_climbJoystick;
    private int m_climbButton;


    public ClimbCommands(ClimbSubsystem climbSubsystem, Joystick climbJoystick, int m_climbButton) {
        this.m_climbSubsystem = climbSubsystem;
        this.m_climbJoystick = climbJoystick;
        this.m_climbButton = m_climbButton;
        addRequirements(climbSubsystem);
    }
    @Override
    public void execute() {
        if (m_climbJoystick.getRawButton( m_climbButton))
            m_climbSubsystem.toggleClimb();
        else 
            m_climbSubsystem.stopClimb();
    }
}
     
