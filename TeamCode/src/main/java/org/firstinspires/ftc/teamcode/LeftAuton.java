package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.DiveActions;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Config
@Autonomous(name="LeftAuton", group="Autonomous")

public class LeftAuton extends LinearOpMode {
@Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-36, -63, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);

        Action AutonLeft = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(-6.25, -45.00), Math.toRadians(90.00))
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
                .splineTo(new Vector2d(-20, -14), Math.toRadians(180.00))
                .build();

        //actions that need to happen on init (if any)



        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        AutonLeft,
                        //DiveActions.Lift.LiftUp(),
                        new DiveActions.sampleDelivery.dump()
                        //new DiveActions.intake.ExtendArm()
                )
        );
    }
}