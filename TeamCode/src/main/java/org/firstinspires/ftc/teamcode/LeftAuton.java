package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name="LeftAuton", preselectTeleOp = "WaldonTeleOp")

public class LeftAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-15, -63, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        DiveActions.Lift Lift = new DiveActions.Lift(hardwareMap);
        DiveActions.SpecimenDelivery SpecimenDelivery = new DiveActions.SpecimenDelivery(hardwareMap);
        DiveActions.SampleDelivery SampleDelivery = new DiveActions.SampleDelivery(hardwareMap);
        DiveActions.LED LED = new DiveActions.LED(hardwareMap);
        DiveActions.Ascend Ascend = new DiveActions.Ascend(hardwareMap);
        DiveActions.Intake Intake = new DiveActions.Intake(hardwareMap);


        Action DeliverSpecimen = drive.actionBuilder(beginPose)
                .lineToY(-32)
                .build();
//
        Action backup = drive.actionBuilder(new Pose2d(-15, -32, Math.toRadians(90)))
                .lineToY(-37)
                .build();
//
        Action pickupSample1 = drive.actionBuilder(new Pose2d(-15, -37, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineTo(new Vector2d(-40,-26),Math.toRadians(180))
                .build();

        Action deliverSample1 = drive.actionBuilder(new Pose2d(-40, -26,Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-58,-58,Math.toRadians(225)),Math.toRadians(180))
                .build();

        Action pickupSample2 = drive.actionBuilder(new Pose2d(-58,-58,Math.toRadians(225)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-48,-26,Math.toRadians(0)),Math.toRadians(180))
                .build();
//
        Action deliverSample2 = drive.actionBuilder(new Pose2d(-48,-26,Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-58,-58,Math.toRadians(225)),Math.toRadians(180))
                .build();

        Action pickupSample3 = drive.actionBuilder(new Pose2d(-58,-58,Math.toRadians(225)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-56,-26,Math.toRadians(0)),Math.toRadians(180))
                .build();

        Action deliverSample3 = drive.actionBuilder(new Pose2d(-56,-26,Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-58,-58,Math.toRadians(225)),Math.toRadians(180))
                .build();


        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        DeliverSpecimen,
                        DiveActions.Lift.liftToHeight(-950),
                        new SleepAction(0.1),
                        new ParallelAction(
                                backup,
                                DiveActions.Lift.autonDown()
                        ),
                        DiveActions.SampleDelivery.dump(),
                        //////////////////////////////////////////////////////////////////
                        // sample 1
                        //////////////////////////////////////////////////////////////////
                        DiveActions.Intake.wristdown(),
                        new ParallelAction(
                                DiveActions.Intake.wheelOn(),
                                pickupSample1
                        ),
                        DiveActions.Intake.wristUp(),
                        DiveActions.Intake.Stop(),
                        deliverSample1,
                        DiveActions.Intake.RevWheel(),
                        DiveActions.Lift.liftToHighBasket(),
                        DiveActions.Intake.Stop(),
                        DiveActions.SampleDelivery.load(),
                        new SleepAction(1),
                        DiveActions.SampleDelivery.dump(),
                        new SleepAction(0.5),
                        DiveActions.Lift.autonDown(),
                        //////////////////////////////////////////////////////////////////
                        // sample 2
                        //////////////////////////////////////////////////////////////////
                        new ParallelAction(
                                DiveActions.Intake.wheelOn(),
                                pickupSample2
                        ),
                        DiveActions.Intake.wristUp(),
                        DiveActions.Intake.Stop(),
                        deliverSample2,
                        DiveActions.Intake.RevWheel(),
                        DiveActions.Lift.liftToHighBasket(),
                        DiveActions.Intake.Stop(),
                        DiveActions.SampleDelivery.load(),
                        new SleepAction(1),
                        DiveActions.SampleDelivery.dump(),
                        new SleepAction(0.5),
                        DiveActions.Lift.autonDown(),
                        //////////////////////////////////////////////////////////////////
                        // sample 3
                        //////////////////////////////////////////////////////////////////
                        new ParallelAction(
                                DiveActions.Intake.wheelOn(),
                                pickupSample3
                        ),
                        DiveActions.Intake.wristUp(),
                        DiveActions.Intake.Stop(),
                        deliverSample3,
                        DiveActions.Intake.RevWheel(),
                        DiveActions.Lift.liftToHighBasket(),
                        DiveActions.Intake.Stop(),
                        DiveActions.SampleDelivery.load(),
                        new SleepAction(1),
                        DiveActions.SampleDelivery.dump(),
                        new SleepAction(0.5),
                        DiveActions.Lift.autonDown()
                )
        );
        //Actions.runBlocking(new SequentialAction(DiveActions.Intake.extendArm()));
    }

}