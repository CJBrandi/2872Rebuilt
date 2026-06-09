package frc.robot.util;

import com.ctre.phoenix6.hardware.traits.CommonDevice;
import java.util.List;

public interface OrchestraInstrumentProvider {
  void addOrchestraInstruments(List<CommonDevice> instruments);
}
