package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommands extends Command {
    private ClimbSubsystem m_climbSubsystem;
    private final Joystick m_climbJoystick;
    private final int m_climbButton;


    public void ClimbCommand(ClimbSubsystem climbSubsystem, Joystick climbJoystick, int m_climbButton) {
        this.m_climbSubsystem = climbSubsystem;
        this.m_climbJoystick = climbJoystick;
        this.m_climbButton = m_climbButton;
        addRequirements(climbSubsystem);
}
@Override
public void execute() {
    if (m_climbJoystick.getRawButton(10))
        m_climbSubsystem.toggleClimb();
    else 
        m_climbSubsystem.stopClimb();
}
