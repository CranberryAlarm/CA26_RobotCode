package frc.robot.simulation;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SimulatableCANSparkMax extends SparkMax {
  SimDeviceSim mCANSparkMaxSim;

  SimDouble mCANSparkMaxSimAppliedOutput;

  public SimulatableCANSparkMax(int deviceId, MotorType type) {
    super(deviceId, type);

    mCANSparkMaxSim = new SimDeviceSim("SPARK MAX ", deviceId);
    mCANSparkMaxSimAppliedOutput = mCANSparkMaxSim.getDouble("Applied Output");

    // TODO: Add other simulation fields
  }

  @Override
  public REVLibError configure(SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
    return super.configure(config, resetMode, persistMode);

    // Just throw away everything for now and say we're ok.
    // return REVLibError.kOk;
  }

  @Override
  public void set(double speed) {
    super.set(speed);

    // TODO: Figure out why this is mad when running on a real robot
    // mCANSparkMaxSimAppliedOutput.set(speed);
  }
}
