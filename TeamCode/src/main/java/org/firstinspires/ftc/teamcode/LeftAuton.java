package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

public final class LeftAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-34.87, -62.99, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //Deliver Specimen to High Chamber
                          //1. raise lift to high chamber
                          //2. drive to submersible
                        .splineTo(new Vector2d(x, y, Math.toRadians(90)))
                          //3. drop lift to deliver
                          //4. pull pin
                          //5. drop all the way
                        //Go to the 3 yellow samples and deliver to high basket
                          //1. Back up to three yellow samples
                          //2. Pick up yellow sample
                            //Turn on intake
                            //Slowly go into sample until we get it
                              //Sense sample??
                            //Hand off to delivery side
                          //3. Raise linear slide to deliver sample to high basket
                          //4. Lower linear slide back down to ground level
                          //5. Repeat step two until out of yellow samples, then move to "Park"
                        //Park
                          //1. Drive over to ascension zone
                          //2. Move linear slide to touch low bar




//                        .splineTo(new Vector2d(-7.62, -39.81), Math.toRadians(90.00))
//                        .splineTo(new Vector2d(-26.29, -34.87), Math.toRadians(188.70))
//                        .splineTo(new Vector2d(-39.17, -25.86), Math.toRadians(176.73))
//                        .splineTo(new Vector2d(-57.84, -57.62), Math.toRadians(225.00))
                        .build());
    }
}