package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public interface SysIdAble {
    Command sysIdQuasistatic(SysIdRoutine.Direction direction);
    Command sysIdDynamic(SysIdRoutine.Direction direction);
}
