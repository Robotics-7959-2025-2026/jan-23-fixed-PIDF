package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Wow, Ayden is in the dictionary but not Adyn??
@Autonomous(name = "Adyn Blue", preselectTeleOp = "TeleOp")
public class ManualAutoBlue extends ManualAuto {
    @Override
    public Paths getPaths() {
        return new PathsBlue(pedro);
    }
}
