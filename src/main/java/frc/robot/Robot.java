/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

 package frc.robot;

 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.Commands;
 import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
 import frc.robot.subsystems.intake.deploy.IntakeDeployHal;
 
 public class Robot {
     public static final String k_canivoreCan = "CANivore";
 
     public final CommandXboxController m_driverController= new CommandXboxController(0);
     public final IntakeDeployHal m_intake;
 
     private Command m_autoCommand = null;
 
     public Robot() {
         configureBindings();
         m_intake = new IntakeDeployHal();
     }
 
     public void update() {}
 
     private void configureBindings() {
         m_driverController.b().onTrue(Commands.runOnce(() -> {
             m_intake.m_motor.setPosition(0);
         }));
     }
 
     public void updateAutoCommand() {}
 
     public Command getAutonomousUCommand() {
         if (m_autoCommand == null) {
             updateAutoCommand();
         }
 
         return m_autoCommand;
     }
 }
 