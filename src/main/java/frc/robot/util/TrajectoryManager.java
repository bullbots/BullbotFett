// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

/** Add your docs here. */
public class TrajectoryManager {
    public static HashMap<String, Trajectory> trajectories;

    public static HashMap<String, Trajectory> generateTrajectories() {
        if (trajectories == null) {
            trajectories = new HashMap<>();
            List<String> path_names = Arrays.asList("/BARREL", "/BOUNCE-1", "/BOUNCE-2", "/BOUNCE-3", "/SLALOM");

            for (var path_name : path_names) {
                var trajPack = TrajectoryPacket.generateTrajectoryPacket(path_name);

                var trajectory =
                TrajectoryGenerator.generateTrajectory(
                    new Pose2d(trajPack.firstX, trajPack.firstY, Rotation2d.fromDegrees(trajPack.start_angle)),
                    trajPack.path_read,
                    new Pose2d(trajPack.lastX, trajPack.lastY, Rotation2d.fromDegrees(trajPack.end_angle)),
                    new TrajectoryConfig(3.0, 3.0));

                trajectories.put(path_name, trajectory);
            }
        }
        return trajectories;
    }
}
