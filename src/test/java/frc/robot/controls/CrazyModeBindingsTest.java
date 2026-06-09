package frc.robot.controls;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class CrazyModeBindingsTest {
  private final CommandScheduler scheduler = CommandScheduler.getInstance();

  @BeforeEach
  void setUp() {
    scheduler.cancelAll();
  }

  @AfterEach
  void tearDown() {
    scheduler.cancelAll();
  }

  @Test
  void orchestraBindingCancelsWhenSongButtonsAreReleased() {
    AtomicBoolean songButtonsHeld = new AtomicBoolean(false);
    EventLoop buttonLoop = new EventLoop();
    SubsystemBase requiredSubsystem = new SubsystemBase() {};
    Command orchestraCommand = Commands.run(() -> {}, requiredSubsystem).ignoringDisable(true);

    CrazyModeBindings.bindOrchestraCommand(
        new Trigger(buttonLoop, songButtonsHeld::get), orchestraCommand);

    buttonLoop.poll();
    scheduler.run();
    assertFalse(orchestraCommand.isScheduled());

    songButtonsHeld.set(true);
    buttonLoop.poll();
    scheduler.run();
    assertTrue(orchestraCommand.isScheduled());

    songButtonsHeld.set(false);
    buttonLoop.poll();
    scheduler.run();
    assertFalse(orchestraCommand.isScheduled());
  }
}
