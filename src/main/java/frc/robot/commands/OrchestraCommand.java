package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.traits.CommonDevice;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Collection;

public class OrchestraCommand extends Command {
  private final Orchestra orchestra = new Orchestra();
  private final boolean loaded;

  public OrchestraCommand(
      Collection<CommonDevice> instruments, String musicFile, Subsystem... requirements) {
    addRequirements(requirements);

    boolean instrumentsOk = true;
    for (var instrument : instruments) {
      var status = orchestra.addInstrument(instrument);
      if (!status.isOK()) {
        instrumentsOk = false;
        DriverStation.reportWarning("Failed to add Orchestra instrument: " + status, false);
      }
    }

    var loadStatus = orchestra.loadMusic(musicFile);
    loaded = instrumentsOk && loadStatus.isOK();
    if (!loadStatus.isOK()) {
      DriverStation.reportWarning(
          "Failed to load Orchestra music " + musicFile + ": " + loadStatus, false);
    }
  }

  @Override
  public void initialize() {
    if (loaded) {
      var status = orchestra.play();
      if (!status.isOK()) {
        DriverStation.reportWarning("Failed to play Orchestra music: " + status, false);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    orchestra.stop();
  }

  @Override
  public boolean isFinished() {
    return !loaded;
  }
}
