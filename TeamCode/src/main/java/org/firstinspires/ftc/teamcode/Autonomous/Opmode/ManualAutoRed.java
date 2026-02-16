package org.firstinspires.ftc.teamcode.Autonomous.Opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.Paths;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.PathsRed;

@Autonomous(name = "Adyn Red", preselectTeleOp = "TeleOp")

public class ManualAutoRed extends ManualAuto {
    @Override
    public Paths getPaths() {
        return new PathsRed(pedro);
    }
}
