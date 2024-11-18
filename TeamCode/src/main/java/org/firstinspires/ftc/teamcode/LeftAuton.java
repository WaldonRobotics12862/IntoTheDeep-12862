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
@Autonomous(name="LeftAuton", group="Autonomous")

public class LeftAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-24, -63, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        DiveActions.Lift Lift = new DiveActions.Lift(hardwareMap);
        DiveActions.SpecimenDelivery SpecimenDelivery = new DiveActions.SpecimenDelivery(hardwareMap);
        DiveActions.SampleDelivery SampleDelivery = new DiveActions.SampleDelivery(hardwareMap);
        DiveActions.LED LED = new DiveActions.LED(hardwareMap);
        DiveActions.Ascend Ascend = new DiveActions.Ascend(hardwareMap);
        DiveActions.Intake Intake = new DiveActions.Intake(hardwareMap);


        Action DeliverSpecimen = drive.actionBuilder(new Pose2d(-15,-63,Math.toRadians(90)))
                .lineToY(-32)
                .build();

        Action backup = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .lineToY(-5)
                .build();

        Action pickupSample1 = drive.actionBuilder(new Pose2d(-15, -65, Math.toRadians(90)))
                //lift up to delivery
                .turnTo(Math.toRadians(350))
                .lineToX(-45)
                .build();

        Action deliverSample1 = drive.actionBuilder(new Pose2d(-45, -24,Math.toRadians(350)))
                .splineTo(new Vector2d(-60,-60),Math.toRadians(225))
                .build();

        Action pickupSample2 = drive.actionBuilder(new Pose2d(-60,-60,Math.toRadians(225)))
                .lineToY(-40)
                .turnTo(Math.toRadians(320))
                .lineToX(-55)
                .build();

        Action deliverSample2 = drive.actionBuilder(new Pose2d(-45,-24,Math.toRadians(320)))
                .splineTo(new Vector2d(-60,-60),Math.toRadians(225))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        DeliverSpecimen,
                        DiveActions.Lift.liftToHeight(Variables.HighChamberDeliver),
                        new SleepAction(0.1),
                        new ParallelAction(
                                backup,
                                DiveActions.Lift.autonDown()
                        ),
                        DiveActions.Intake.wristdown(),
                        new ParallelAction(
                                DiveActions.Intake.wheelOn(),
                                pickupSample1
                        ),
                        DiveActions.Intake.Stop(),
                        DiveActions.Intake.wristUp(),
                        deliverSample1,
                        DiveActions.Intake.RevWheel(),
                        new SleepAction(0.5),
                        DiveActions.Lift.liftToHeight(Variables.HighBasket),
                        DiveActions.Intake.Stop(),
                        DiveActions.SampleDelivery.load(),
                        new SleepAction(1),
                        DiveActions.SampleDelivery.dump(),
                        DiveActions.Lift.autonDown(),
                        DiveActions.Intake.wristdown(),
                        new ParallelAction(
                            DiveActions.Intake.wheelOn(),
                            pickupSample2
                        ),
                        DiveActions.Intake.Stop(),
                        DiveActions.Intake.wristUp(),
                        deliverSample2,
                        DiveActions.Intake.RevWheel(),
                        new SleepAction(0.5),
                        DiveActions.Lift.liftToHeight(Variables.HighBasket),
                        DiveActions.Intake.Stop(),
                        DiveActions.SampleDelivery.load(),
                        new SleepAction(1),
                        DiveActions.SampleDelivery.dump(),
                        DiveActions.Lift.autonDown()
                )
        );
        //Actions.runBlocking(new SequentialAction(DiveActions.Intake.extendArm()));
    }

}