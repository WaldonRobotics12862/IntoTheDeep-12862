package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(SparkFunOTOSDrive.class)) {
            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);

            waitForStart();
            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                    .splineTo(new Vector2d(-34.68, -37.43), Math.toRadians(86.19))
                    .splineToConstantHeading(new Vector2d(-11.33, -37.66), Math.toRadians(-0.56))
                    .splineTo(new Vector2d(9.04, -39.49), Math.toRadians(0.00))
                    .splineTo(new Vector2d(-10.42, -60.10), Math.toRadians(226.64))
                    .build());
            //Actions.runBlocking(
            //    drive.actionBuilder(beginPose)
            //            .splineTo(new Vector2d(30, 30), Math.PI / 2)
            //            .splineTo(new Vector2d(0, 60), Math.PI)
            //            .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
