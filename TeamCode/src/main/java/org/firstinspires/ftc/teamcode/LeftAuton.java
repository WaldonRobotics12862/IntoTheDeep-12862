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
        Pose2d beginPose = new Pose2d(-36, -63, Math.toRadians(90));
        Pose2d samplePickUp = new Pose2d(-25, -36, Math.toRadians(160));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        DiveActions.Lift Lift = new DiveActions.Lift(hardwareMap);
        DiveActions.SpecimenDelivery SpecimenDelivery = new DiveActions.SpecimenDelivery(hardwareMap);
        DiveActions.SampleDelivery SampleDelivery = new DiveActions.SampleDelivery(hardwareMap);
        DiveActions.LED LED = new DiveActions.LED(hardwareMap);
        DiveActions.Ascend Ascend = new DiveActions.Ascend(hardwareMap);
        DiveActions.Intake Intake = new DiveActions.Intake(hardwareMap);


        Action DriveIntoSample = drive.actionBuilder(samplePickUp)
                .lineToX(-24)
                .build();

        Action AutonLeft = drive.actionBuilder(beginPose)
                //lift up to delivery
                .splineTo(new Vector2d(-5, -33.00), Math.toRadians(90.00))
                //pull down on sample
                .lineToY(-48)
                //.turn(Math.toRadians(0))
                //deploy
                .splineTo(new Vector2d(-25, -36), Math.toRadians(160))
                //.waitSeconds(3)
                //.setTangent(0)
                //.splineToConstantHeading(new Vector2d(-6.25, -50), Math.toRadians(90.00))
                //.splineToConstantHeading(new Vector2d(-55.25, -50), Math.toRadians(90.00))
                //.splineToConstantHeading(new Vector2d(-55.25, -46.0), Math.toRadians(90.00))
                //.splineTo(new Vector2d(-58.5, -58), Math.toRadians(225))
                //.splineTo(new Vector2d(-59, -47), Math.toRadians(90))
                //.turn(Math.toRadians(215))
                //.splineTo(new Vector2d(-67, -47), Math.toRadians(90))
                //.splineToConstantHeading(new Vector2d(-67, -14), Math.toRadians(90))
                //.splineTo(new Vector2d(-20, -14), Math.toRadians(180.00))
                .build();

        //actions that need to happen on init (if any)
        //example below
        //Actions.runBlocking(claw.closeClaw());

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
//        Actions.runBlocking(DiveActions.Intake.ExtendArm());
//        Actions.runBlocking(DiveActions.Intake.WheelOn());

                new SequentialAction(
                        AutonLeft,
                        new DiveActions.Intake.DebugAction(this.telemetry, "Starting arm"),
                        DiveActions.Intake.extendArm(telemetry),
//                        new DiveActions.Intake.DebugAction(this.telemetry, "Starting wheel"),
                        new SleepAction(2 ),
                        new ParallelAction(
                            DiveActions.Intake.wheelOn(),
                            DriveIntoSample
                        )
                        //new SleepAction(5),
                        //DiveActions.Intake.WheelOn()
                        //new DiveActions.intake.ExtendArm()
                )
        );
        //Actions.runBlocking(new SequentialAction(DiveActions.Intake.extendArm()));
    }

}