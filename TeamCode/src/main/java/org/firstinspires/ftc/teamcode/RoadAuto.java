package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import drive.SampleMecanumDrive;
@Autonomous(name="RoadAuto")
public class RoadAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory hello = drive.trajectoryBuilder(new Pose2d())
                .forward(10)
                .build();

        Trajectory yes = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(12)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(hello);
        drive.followTrajectory(yes);
    }
}