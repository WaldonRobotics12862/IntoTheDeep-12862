package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="RightAuton")
public final class RightAuton extends LinearOpMode {
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
                        //.splineTo(new Vector2d(x, y, Math.toRadians(90)))
                          //3. drop lift to deliver
                          //4. pull pin
                          //5. drop all the way
                        //Go over to red/blue samples and deliver to observation zone
                          // 1.Drive up to red/blue sample area
                          // 2.Pick up a red/blue sample
                            // 1.Activate intake
                            // 2.Slowly go into sample until picked up
                            // 3.Transfer sample to delivery side
                            // 4.Activate delivery to drop sample into the observation zone
                            // moves backwards out of observation zone
                            // the human player takes the sample and then puts on the clip then drops in  certain area
                            // the robot moves back into observation zone
                            // activate intake to pick up specimen
                            // transfer intake to deliver
                            // drive to high chamber
                            // raise linear slide
                            // activate deliver to put specimen on high chamber
                            // back up off linear slide
                            // lower linear slide
                            // repeat back to step 1.


                            // Drive back to samples then Repeat to step one until all samples gone
                        //Park
                            //drive to observation zone




//                        .splineTo(new Vector2d(-7.62, -39.81), Math.toRadians(90.00))
//                        .splineTo(new Vector2d(-26.29, -34.87), Math.toRadians(188.70))
//                        .splineTo(new Vector2d(-39.17, -25.86), Math.toRadians(176.73))
//                        .splineTo(new Vector2d(-57.84, -57.62), Math.toRadians(225.00))
                        .build());
    }
}