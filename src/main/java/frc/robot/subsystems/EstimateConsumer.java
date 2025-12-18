package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class EstimateConsumer {
  Pose2d pose;
  double timestamp;
  Matrix<N3, N1> estimationStdDevs;
  boolean initialized = false;
  boolean stale = true;

  public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
    this.pose = pose;
    this.timestamp = timestamp;
    this.estimationStdDevs = estimationStdDevs;
    initialized = true;
    stale = false;
  }

  public void setStale() {
    stale = true;
  }

  public boolean isInitialized() {
    return initialized;
  }

  public boolean isStale() {
    return stale;
  }

  public Pose2d getPose2d() {
    return pose;
  }

  public Matrix<N3, N1> getStdDevs() {
    return estimationStdDevs;
  }

  public double getTimeStamp() {
    return timestamp;
  }
}
