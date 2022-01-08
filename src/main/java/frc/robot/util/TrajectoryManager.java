// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

/** Add your docs here. */
public class TrajectoryManager {

    private static HashMap<String, Trajectory> trajectories;

    // trajectories are now protected by ReentrantLock
    private static ReentrantLock trajectoriesLock = new ReentrantLock();

    public static void generateTrajectories() {

        Thread genTrajectoriesThread = new Thread(() -> {

            if (trajectories == null) {
                System.out.println("Info: Trajectories loading...");
                trajectoriesLock.lock();
    
                trajectories = new HashMap<>();
    
                List<String> pathNames = new ArrayList<>();
    
                File deployDirectory = Filesystem.getDeployDirectory();
                File[] listOfFiles = deployDirectory.listFiles();
    
                for (File file : listOfFiles) {
                    pathNames.add("/" + file.getName());
                    // No filter is needed for now since only files in deploy directory are path files.
                }
    
                // List<String> path_names = Arrays.asList("/BARREL", "/SLALOM");
    
                for (var pathName : pathNames) {
                    // System.out.println(String.format("Adding PathName: %s", pathName));
                    var trajPack = TrajectoryPacket.generateTrajectoryPacket(pathName);
    
                    var trajectory =
                    TrajectoryGenerator.generateTrajectory(
                        new Pose2d(trajPack.firstX, trajPack.firstY, Rotation2d.fromDegrees(trajPack.start_angle)),
                        trajPack.path_read,
                        new Pose2d(trajPack.lastX, trajPack.lastY, Rotation2d.fromDegrees(trajPack.end_angle)),
                        new TrajectoryConfig(2.0, 4.0));
    
                    trajectories.put(pathName, trajectory);
                }
                trajectoriesLock.unlock();
                System.out.println("Info: Trajectories loaded");
            }

        });

        genTrajectoriesThread.setDaemon(true);
        genTrajectoriesThread.start();
    }

    public static HashMap<String, Trajectory> getTrajectories() {
        HashMap<String, Trajectory> cur_trajectories = null;

        if (trajectories != null && trajectoriesLock.tryLock()) {
            cur_trajectories = trajectories;
            trajectoriesLock.unlock();
        }

        return cur_trajectories;
    }
}
