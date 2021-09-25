package org.firstinspires.ftc.teamcode.drive.Templates;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DualPad;
import org.firstinspires.ftc.teamcode.util.DualPadMimicking;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

@TeleOp
public class mecDriveMimicking extends LinearOpMode
{
    //Create Robot Hardware Object
    RobotHardware robot = new RobotHardware();
    DualPadMimicking gpad = new DualPadMimicking();
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);

        gpad.createFile("gpadValues");

        //Initialize Hardware and reverse motors
        telemetry.addData("Status", "Initialized");

        waitForStart();

        runtime.reset();
        while(opModeIsActive()) {
            gpad.mergePads(gamepad1, gamepad2, runtime.nanoseconds());

            double jy = -gpad.left_stick_y - gpad.right_stick_y; // forward
            double jx = gpad.right_stick_x;  // strafing
            double jw = gpad.left_stick_x;   // turning

            robot.driveYXW(jy,jx,jw);

            telemetry.update();
        }
    }

}
