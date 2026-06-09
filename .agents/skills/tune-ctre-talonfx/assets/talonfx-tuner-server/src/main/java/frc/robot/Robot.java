package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executors;

public class Robot extends TimedRobot {
  private final Object runLock = new Object();
  private final NeutralOut neutral = new NeutralOut();
  private TalonFX leader;
  private final List<TalonFX> followers = new ArrayList<>();
  private StatusSignal<?>[] healthSignals = new StatusSignal<?>[0];
  private HttpServer server;
  private Map<String, Object> latestData = Map.of("status", "no runs yet");
  private Map<String, Object> bootConfig = Map.of();
  private boolean configured = false;

  @Override
  public void robotInit() {
    try {
      bootConfig = readBootConfig();
      configureDevices(bootConfig);
      startHttpServer(intValue(bootConfig, "httpPort", 5805));
      System.out.println("[talonfx-tuner] ready on port " + intValue(bootConfig, "httpPort", 5805));
      System.out.println(
          "[talonfx-tuner] Phoenix Diagnostics should also be running after TalonFX construction.");
    } catch (Exception ex) {
      DriverStation.reportError(
          "[talonfx-tuner] startup failed: " + ex.getMessage(), ex.getStackTrace());
      throw new RuntimeException(ex);
    }
  }

  @Override
  public void disabledInit() {
    neutralize();
  }

  @Override
  public void close() {
    neutralize();
    if (server != null) {
      server.stop(0);
    }
    super.close();
  }

  private Map<String, Object> readBootConfig() throws IOException {
    Path configPath = Filesystem.getDeployDirectory().toPath().resolve("talonfx-tuner-config.json");
    return Json.parseObject(Files.readString(configPath, StandardCharsets.UTF_8));
  }

  @SuppressWarnings("unchecked")
  private void configureDevices(Map<String, Object> config) {
    int leaderId = intValue(config, "leaderId", -1);
    if (leaderId < 0) {
      throw new IllegalArgumentException("leaderId is required");
    }
    String canBusName = stringValue(config, "canBus", "rio");
    CANBus canBus = new CANBus(canBusName);
    leader = new TalonFX(leaderId, canBus);
    applyBaseConfig(leader, config);

    followers.clear();
    Object rawFollowers = config.get("followers");
    if (rawFollowers instanceof List<?> followerList) {
      for (Object raw : followerList) {
        if (!(raw instanceof Map<?, ?> followerMap)) {
          continue;
        }
        int id = intValue((Map<String, Object>) followerMap, "id", -1);
        if (id < 0) {
          continue;
        }
        String alignment = stringValue((Map<String, Object>) followerMap, "alignment", "Aligned");
        TalonFX follower = new TalonFX(id, canBus);
        follower.setControl(new Follower(leaderId, motorAlignment(alignment)));
        followers.add(follower);
      }
    }

    healthSignals =
        new StatusSignal<?>[] {
          leader.getVersion(false),
          leader.getPosition(false),
          leader.getVelocity(false),
          leader.getSupplyVoltage(false),
          leader.getStatorCurrent(false),
          leader.getFault_UnlicensedFeatureInUse(false)
        };
    BaseStatusSignal.setUpdateFrequencyForAll(50, healthSignals);
    ParentDevice.optimizeBusUtilizationForAll(allDevices());
    configured = true;
    neutralize();
  }

  private TalonFX[] allDevices() {
    TalonFX[] devices = new TalonFX[followers.size() + 1];
    devices[0] = leader;
    for (int i = 0; i < followers.size(); i++) {
      devices[i + 1] = followers.get(i);
    }
    return devices;
  }

  private void applyBaseConfig(TalonFX motor, Map<String, Object> source) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    applyBaseFields(cfg, source);
    StatusCode status = motor.getConfigurator().apply(cfg, 0.5);
    if (!status.isOK()) {
      throw new IllegalStateException("Failed to apply base TalonFX config: " + status);
    }
  }

  private void startHttpServer(int port) throws IOException {
    server = HttpServer.create(new InetSocketAddress(port), 0);
    server.createContext("/health", this::handleHealth);
    server.createContext("/run", this::handleRun);
    server.createContext("/stop", this::handleStop);
    server.createContext("/data/latest", this::handleLatest);
    server.setExecutor(Executors.newCachedThreadPool());
    server.start();
  }

  private void handleHealth(HttpExchange exchange) throws IOException {
    if (!"GET".equals(exchange.getRequestMethod())) {
      send(exchange, 405, Map.of("error", "GET required"));
      return;
    }
    send(exchange, 200, health());
  }

  private void handleLatest(HttpExchange exchange) throws IOException {
    if (!"GET".equals(exchange.getRequestMethod())) {
      send(exchange, 405, Map.of("error", "GET required"));
      return;
    }
    send(exchange, 200, latestData);
  }

  private void handleStop(HttpExchange exchange) throws IOException {
    neutralize();
    send(exchange, 200, Map.of("stopped", true, "health", health()));
  }

  private void handleRun(HttpExchange exchange) throws IOException {
    if (!"POST".equals(exchange.getRequestMethod())) {
      send(exchange, 405, Map.of("error", "POST required"));
      return;
    }
    if (!configured) {
      send(exchange, 409, Map.of("error", "server is not configured"));
      return;
    }

    try {
      String body = new String(exchange.getRequestBody().readAllBytes(), StandardCharsets.UTF_8);
      Map<String, Object> plan = Json.parseObject(body);
      Map<String, Object> data;
      synchronized (runLock) {
        data = runPlan(plan);
        latestData = data;
      }
      send(exchange, 200, data);
    } catch (Exception ex) {
      neutralize();
      latestData = Map.of("ok", false, "error", ex.getMessage(), "health", health());
      send(exchange, 400, latestData);
    }
  }

  @SuppressWarnings("unchecked")
  private Map<String, Object> runPlan(Map<String, Object> plan) {
    requireEnabled();
    String mode = stringValue(plan, "mode", "");
    double durationSec = doubleValue(plan, "durationSec", 0);
    if (durationSec <= 0 || durationSec > 30) {
      throw new IllegalArgumentException("durationSec must be > 0 and <= 30");
    }
    List<Double> setpoints = doubleList(plan.get("setpoints"));
    if (setpoints.isEmpty()) {
      throw new IllegalArgumentException("setpoints must contain at least one value");
    }
    Map<String, Object> safety = mapValue(plan, "safety");
    validateSetpoints(mode, setpoints, safety);

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    applyBaseFields(cfg, bootConfig);
    applyGains(cfg, mapValue(plan, "gains"));
    applyLimits(cfg, safety);
    applyMotionMagic(cfg, mapValue(plan, "motionMagic"));
    if (plan.containsKey("sensorToMechanismRatio")) {
      cfg.Feedback.SensorToMechanismRatio = doubleValue(plan, "sensorToMechanismRatio", 1.0);
    }
    if (plan.containsKey("rotorToSensorRatio")) {
      cfg.Feedback.RotorToSensorRatio = doubleValue(plan, "rotorToSensorRatio", 1.0);
    }
    StatusCode applyStatus = leader.getConfigurator().apply(cfg, 0.5);
    if (!applyStatus.isOK()) {
      throw new IllegalStateException("TalonFX config apply failed: " + applyStatus);
    }

    double start = Timer.getFPGATimestamp();
    double samplePeriod = Math.max(0.005, doubleValue(plan, "samplePeriodSec", 0.02));
    List<Object> samples = new ArrayList<>();
    String abortReason = "";
    try {
      while (Timer.getFPGATimestamp() - start < durationSec) {
        double elapsed = Timer.getFPGATimestamp() - start;
        double setpoint = setpointAt(setpoints, elapsed, durationSec);
        sendControl(mode, setpoint);
        BaseStatusSignal.refreshAll(healthSignals);
        Map<String, Object> sample = sample(mode, setpoint, elapsed);
        samples.add(sample);
        abortReason = abortReason(sample, safety);
        if (!abortReason.isEmpty()) {
          break;
        }
        if (!DriverStation.isEnabled()) {
          abortReason = "Driver Station disabled during run";
          break;
        }
        sleep(samplePeriod);
      }
    } finally {
      neutralize();
    }

    Map<String, Object> result = new LinkedHashMap<>();
    result.put("ok", abortReason.isEmpty());
    result.put("abortReason", abortReason);
    result.put("mode", mode);
    result.put("durationSec", Timer.getFPGATimestamp() - start);
    result.put("sampleCount", samples.size());
    result.put("metrics", metrics(samples));
    result.put("samples", samples);
    result.put("health", health());
    return result;
  }

  private void requireEnabled() {
    double deadline = Timer.getFPGATimestamp() + 2.0;
    while (!DriverStation.isEnabled() && Timer.getFPGATimestamp() < deadline) {
      sleep(0.02);
    }
    if (!DriverStation.isEnabled()) {
      throw new IllegalStateException("Driver Station did not enable before run");
    }
  }

  private void sendControl(String mode, double setpoint) {
    switch (mode) {
      case "velocity-voltage" -> leader.setControl(new VelocityVoltage(setpoint));
      case "velocity-torque-current" -> leader.setControl(new VelocityTorqueCurrentFOC(setpoint));
      case "motion-magic-voltage" -> leader.setControl(new MotionMagicVoltage(setpoint));
      case "motion-magic-torque-current" -> leader.setControl(
          new MotionMagicTorqueCurrentFOC(setpoint));
      default -> throw new IllegalArgumentException("Unsupported mode: " + mode);
    }
  }

  private Map<String, Object> sample(String mode, double setpoint, double elapsed) {
    boolean positionMode = mode.startsWith("motion-magic");
    double actual =
        positionMode
            ? leader.getPosition(false).getValueAsDouble()
            : leader.getVelocity(false).getValueAsDouble();
    double error = setpoint - actual;
    Map<String, Object> sample = new LinkedHashMap<>();
    sample.put("t", elapsed);
    sample.put("setpoint", setpoint);
    sample.put("actual", actual);
    sample.put("error", error);
    sample.put("positionRot", leader.getPosition(false).getValueAsDouble());
    sample.put("velocityRps", leader.getVelocity(false).getValueAsDouble());
    sample.put("motorVoltage", leader.getMotorVoltage(false).getValueAsDouble());
    sample.put("statorCurrent", leader.getStatorCurrent(false).getValueAsDouble());
    sample.put("supplyCurrent", leader.getSupplyCurrent(false).getValueAsDouble());
    sample.put("supplyVoltage", leader.getSupplyVoltage(false).getValueAsDouble());
    sample.put("batteryVoltage", RobotController.getBatteryVoltage());
    sample.put("deviceTempC", leader.getDeviceTemp(false).getValueAsDouble());
    sample.put(
        "unlicensedFault",
        Boolean.TRUE.equals(leader.getFault_UnlicensedFeatureInUse(false).getValue()));
    sample.put(
        "undervoltageFault", Boolean.TRUE.equals(leader.getFault_Undervoltage(false).getValue()));
    sample.put(
        "statorLimitFault", Boolean.TRUE.equals(leader.getFault_StatorCurrLimit(false).getValue()));
    sample.put(
        "supplyLimitFault", Boolean.TRUE.equals(leader.getFault_SupplyCurrLimit(false).getValue()));
    sample.put(
        "forwardLimitFault",
        Boolean.TRUE.equals(leader.getFault_ForwardHardLimit(false).getValue()));
    sample.put(
        "reverseLimitFault",
        Boolean.TRUE.equals(leader.getFault_ReverseHardLimit(false).getValue()));
    return sample;
  }

  private String abortReason(Map<String, Object> sample, Map<String, Object> safety) {
    if (Boolean.TRUE.equals(sample.get("unlicensedFault"))) {
      return "UnlicensedFeatureInUse fault";
    }
    if (doubleEntry(sample, "supplyVoltage") < doubleValue(safety, "minSupplyVoltage", 6.8)) {
      return "supply voltage below minimum";
    }
    if (doubleEntry(sample, "deviceTempC") > doubleValue(safety, "maxDeviceTempC", 85)) {
      return "device temperature above maximum";
    }
    if (Math.abs(doubleEntry(sample, "motorVoltage"))
        > doubleValue(safety, "maxAbsVoltage", 16.0) + 0.1) {
      return "motor voltage above requested cap";
    }
    if (Math.abs(doubleEntry(sample, "statorCurrent"))
        > doubleValue(safety, "maxAbsStatorCurrent", 800) + 5) {
      return "stator current above requested cap";
    }
    if (Math.abs(doubleEntry(sample, "supplyCurrent"))
        > doubleValue(safety, "maxAbsSupplyCurrent", 800) + 5) {
      return "supply current above requested cap";
    }
    return "";
  }

  private Map<String, Object> metrics(List<Object> rawSamples) {
    double sumSq = 0;
    double maxAbsError = 0;
    double maxStator = 0;
    double maxSupply = 0;
    double minSupplyVoltage = Double.POSITIVE_INFINITY;
    int count = 0;
    for (Object raw : rawSamples) {
      @SuppressWarnings("unchecked")
      Map<String, Object> sample = (Map<String, Object>) raw;
      double error = Math.abs(doubleEntry(sample, "error"));
      sumSq += error * error;
      maxAbsError = Math.max(maxAbsError, error);
      maxStator = Math.max(maxStator, Math.abs(doubleEntry(sample, "statorCurrent")));
      maxSupply = Math.max(maxSupply, Math.abs(doubleEntry(sample, "supplyCurrent")));
      minSupplyVoltage = Math.min(minSupplyVoltage, doubleEntry(sample, "supplyVoltage"));
      count++;
    }
    Map<String, Object> metrics = new LinkedHashMap<>();
    metrics.put("rmsError", count == 0 ? 0 : Math.sqrt(sumSq / count));
    metrics.put("maxAbsError", maxAbsError);
    metrics.put("peakStatorCurrent", maxStator);
    metrics.put("peakSupplyCurrent", maxSupply);
    metrics.put("minSupplyVoltage", count == 0 ? 0 : minSupplyVoltage);
    return metrics;
  }

  private Map<String, Object> health() {
    Map<String, Object> health = new LinkedHashMap<>();
    health.put("ready", configured);
    health.put("enabled", DriverStation.isEnabled());
    health.put("leaderId", bootConfig.get("leaderId"));
    health.put("canBus", bootConfig.get("canBus"));
    if (leader != null) {
      StatusCode status = BaseStatusSignal.refreshAll(healthSignals);
      health.put("status", status.toString());
      health.put("statusOk", status.isOK());
      health.put("version", leader.getVersion(false).getValue());
      health.put("positionRot", leader.getPosition(false).getValueAsDouble());
      health.put("velocityRps", leader.getVelocity(false).getValueAsDouble());
      health.put("supplyVoltage", leader.getSupplyVoltage(false).getValueAsDouble());
      health.put("statorCurrent", leader.getStatorCurrent(false).getValueAsDouble());
      health.put(
          "unlicensedFault",
          Boolean.TRUE.equals(leader.getFault_UnlicensedFeatureInUse(false).getValue()));
    }
    return health;
  }

  private void applyGains(TalonFXConfiguration cfg, Map<String, Object> gains) {
    cfg.Slot0.kP = doubleValue(gains, "kP", 0);
    cfg.Slot0.kI = doubleValue(gains, "kI", 0);
    cfg.Slot0.kD = doubleValue(gains, "kD", 0);
    cfg.Slot0.kS = doubleValue(gains, "kS", 0);
    cfg.Slot0.kV = doubleValue(gains, "kV", 0);
    cfg.Slot0.kA = doubleValue(gains, "kA", 0);
    cfg.Slot0.kG = doubleValue(gains, "kG", 0);
  }

  private void applyBaseFields(TalonFXConfiguration cfg, Map<String, Object> source) {
    cfg.MotorOutput.NeutralMode = neutralMode(stringValue(source, "neutralMode", "Brake"));
    cfg.MotorOutput.Inverted =
        invertedValue(stringValue(source, "leaderInverted", "CounterClockwise_Positive"));
    if (source.containsKey("sensorToMechanismRatio")) {
      cfg.Feedback.SensorToMechanismRatio = doubleValue(source, "sensorToMechanismRatio", 1.0);
    }
    if (source.containsKey("rotorToSensorRatio")) {
      cfg.Feedback.RotorToSensorRatio = doubleValue(source, "rotorToSensorRatio", 1.0);
    }
  }

  private void applyLimits(TalonFXConfiguration cfg, Map<String, Object> safety) {
    double maxVoltage = doubleValue(safety, "maxAbsVoltage", 6);
    cfg.Voltage.PeakForwardVoltage = Math.abs(maxVoltage);
    cfg.Voltage.PeakReverseVoltage = -Math.abs(maxVoltage);

    double stator = doubleValue(safety, "maxAbsStatorCurrent", 40);
    cfg.CurrentLimits.StatorCurrentLimit = Math.abs(stator);
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    double supply = doubleValue(safety, "maxAbsSupplyCurrent", 40);
    cfg.CurrentLimits.SupplyCurrentLimit = Math.abs(supply);
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    double torque = doubleValue(safety, "maxAbsTorqueCurrent", stator);
    cfg.TorqueCurrent.PeakForwardTorqueCurrent = Math.abs(torque);
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = -Math.abs(torque);
  }

  private void applyMotionMagic(TalonFXConfiguration cfg, Map<String, Object> motionMagic) {
    cfg.MotionMagic.MotionMagicCruiseVelocity = doubleValue(motionMagic, "cruiseVelocity", 0);
    cfg.MotionMagic.MotionMagicAcceleration = doubleValue(motionMagic, "acceleration", 0);
    cfg.MotionMagic.MotionMagicJerk = doubleValue(motionMagic, "jerk", 0);
  }

  private void validateSetpoints(String mode, List<Double> setpoints, Map<String, Object> safety) {
    boolean positionMode = mode.startsWith("motion-magic");
    double min =
        doubleValue(safety, positionMode ? "minPosition" : "minVelocity", -Double.MAX_VALUE);
    double max =
        doubleValue(safety, positionMode ? "maxPosition" : "maxVelocity", Double.MAX_VALUE);
    if (positionMode
        && (!safety.containsKey("minPosition") || !safety.containsKey("maxPosition"))) {
      throw new IllegalArgumentException("Motion Magic tests require minPosition and maxPosition");
    }
    if (!positionMode
        && (!safety.containsKey("minVelocity") || !safety.containsKey("maxVelocity"))) {
      throw new IllegalArgumentException("Velocity tests require minVelocity and maxVelocity");
    }
    for (double setpoint : setpoints) {
      if (setpoint < min || setpoint > max) {
        throw new IllegalArgumentException(
            "setpoint " + setpoint + " is outside [" + min + ", " + max + "]");
      }
    }
  }

  private double setpointAt(List<Double> setpoints, double elapsed, double durationSec) {
    int index =
        Math.min(setpoints.size() - 1, (int) Math.floor(elapsed / durationSec * setpoints.size()));
    return setpoints.get(index);
  }

  private void neutralize() {
    if (leader != null) {
      leader.setControl(neutral);
    }
    for (TalonFX follower : followers) {
      follower.setControl(neutral);
    }
  }

  private void send(HttpExchange exchange, int status, Object body) throws IOException {
    byte[] bytes = Json.stringify(body).getBytes(StandardCharsets.UTF_8);
    exchange.getResponseHeaders().set("Content-Type", "application/json");
    exchange.sendResponseHeaders(status, bytes.length);
    exchange.getResponseBody().write(bytes);
    exchange.close();
  }

  private void sleep(double seconds) {
    try {
      Thread.sleep(Math.max(1, Math.round(seconds * 1000.0)));
    } catch (InterruptedException ex) {
      Thread.currentThread().interrupt();
      throw new RuntimeException(ex);
    }
  }

  private static Map<String, Object> mapValue(Map<String, Object> source, String key) {
    Object raw = source.get(key);
    if (raw instanceof Map<?, ?> map) {
      Map<String, Object> result = new LinkedHashMap<>();
      for (var entry : map.entrySet()) {
        result.put(String.valueOf(entry.getKey()), entry.getValue());
      }
      return result;
    }
    return Map.of();
  }

  private static List<Double> doubleList(Object raw) {
    List<Double> result = new ArrayList<>();
    if (raw instanceof List<?> list) {
      for (Object item : list) {
        if (item instanceof Number number) {
          result.add(number.doubleValue());
        }
      }
    }
    return result;
  }

  private static int intValue(Map<String, Object> source, String key, int fallback) {
    Object value = source.get(key);
    return value instanceof Number number ? number.intValue() : fallback;
  }

  private static double doubleValue(Map<String, Object> source, String key, double fallback) {
    Object value = source.get(key);
    return value instanceof Number number ? number.doubleValue() : fallback;
  }

  private static double doubleEntry(Map<String, Object> source, String key) {
    return doubleValue(source, key, 0.0);
  }

  private static String stringValue(Map<String, Object> source, String key, String fallback) {
    Object value = source.get(key);
    return value == null ? fallback : String.valueOf(value);
  }

  private static NeutralModeValue neutralMode(String value) {
    return "Coast".equalsIgnoreCase(value) ? NeutralModeValue.Coast : NeutralModeValue.Brake;
  }

  private static InvertedValue invertedValue(String value) {
    return "Clockwise_Positive".equalsIgnoreCase(value) || "clockwise".equalsIgnoreCase(value)
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
  }

  private static MotorAlignmentValue motorAlignment(String value) {
    return "Opposed".equalsIgnoreCase(value)
        ? MotorAlignmentValue.Opposed
        : MotorAlignmentValue.Aligned;
  }
}
