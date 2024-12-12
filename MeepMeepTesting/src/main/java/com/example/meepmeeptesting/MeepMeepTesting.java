package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        //RIGHT AUTON 2 VISUALIZATION PATH
        //This delivers one, then goes and pushes the specimen over into the observations zone and immeadiatly picks up a second
        //specimen that is already with the human player.
        //The human player builds a 3rd specimen while the second is being delivered
        //This path is only used if our partner does NOT load a specimen but this is our preferred path
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(15, -63, Math.toRadians(90)))
//                //Deliver 1
//                .lineToY(-32)
//                //backup2
//                .lineToY(-33)
//                //Pickup 2
//                .lineToY(-38)
//                .setTangent(0)
//                //.splineToLinearHeading(new Pose2d(18,-38, Math.toRadians(90)), 0) // this should strafe right
//                .strafeTo(new Vector2d(20,-38))
//                .splineTo(new Vector2d(34,-12), Math.toRadians(90))
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(58,-55, Math.toRadians(180)),Math.toRadians(45)) // push the sample over to the observation zone
//                .setTangent(0)
//                .lineToX(55)
//                .splineToLinearHeading(new Pose2d(40, -45, Math.toRadians(-90)), 0)
//                .setTangent(Math.toRadians(270))
//                //Pickup 22
//                .lineToY(-66)
//                //Deliver 2
//                .lineToY(-58)
//                .splineToSplineHeading(new Pose2d(6,-32,Math.toRadians(90)),Math.toRadians(90))
//                .lineToY(-28)
//                //Pickup 3
//                .lineToY(-38)
//                .splineToSplineHeading(new Pose2d(40,-60,Math.toRadians(270)),Math.toRadians(-90))
//                .lineToY(-66)
//                //Deliver 3
//                .lineToY(-58)
//                .splineToSplineHeading(new Pose2d(3,-35,Math.toRadians(90)),Math.toRadians(90))
//                .lineToY(-28)
//                //Park
//                .lineToY(-38)
//                .setTangent(0)
////                .splineTo(new Vector2d(40,-50),Math.toRadians(150))
//                .splineToLinearHeading(new Pose2d(40,-50,Math.toRadians(150)),Math.toRadians(90))
//                .build()
//        );


        myBot.runAction(myBot.getDrive()
                .actionBuilder(new Pose2d(-55, -59, Math.toRadians(225)))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading((new Pose2d(-58,-43,Math.toRadians(240))),Math.toRadians(90))
                // strafe straight in X
                .setTangent(120)
                .splineToConstantHeading(new Vector2d(-56,-43),Math.toRadians(240))
                .build()
        );


//        myBot.runAction(myBot.getDrive()
//                .actionBuilder(new Pose2d(-58.25, -58.25, Math.toRadians(225)))
//                .setTangent(Math.toRadians(-90))
//                .splineTo(new Vector2d(-39,-32),Math.toRadians(180))
//                .build()
//        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}