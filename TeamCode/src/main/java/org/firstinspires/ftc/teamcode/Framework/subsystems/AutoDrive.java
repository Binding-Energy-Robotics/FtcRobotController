package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

public class AutoDrive extends SubsystemBase {

    private SampleMecanumDrive drive;

    public AutoDrive(HardwareMap hw, Pose2d initialPosition){
        this.drive = new SampleMecanumDrive(hw);
        this.drive.setPoseEstimate(initialPosition);
    }

    public AutoDrive(HardwareMap hw){
        this(hw, new Pose2d());
    }

    public SampleMecanumDrive getDrive(){
        return this.drive;
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return drive.trajectoryBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startTangent) {
        return drive.trajectoryBuilder(startPose, startTangent);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return drive.trajectoryBuilder(startPose, reversed);
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }

    public void update() {
        drive.update();
    }
}
