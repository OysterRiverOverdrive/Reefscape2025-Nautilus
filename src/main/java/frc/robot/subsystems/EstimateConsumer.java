package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class EstimateConsumer {
  Pose2d pose;
  double timestamp;
  Matrix<N3, N1> estimationStdDevs;

  public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
    this.pose = pose;
    this.timestamp = timestamp;
    this.estimationStdDevs = estimationStdDevs;
  }

  public Pose2d getPose2d() {
    return pose;
  }
}
