// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    /*DUTY CYLCE ABSOLUTE ENCODER 
    * 
    * (CONECTAOD A LOS PUERTOS DE LA DIO)
    */
    int DutyPort = 1; // Puerto de lectura de DIO

    double DutyCycleEncOffset = 0.0; //Desface del encoder (EN GRADOS)

    /* ANALOG INPUT ABSOLUTE ENCODER
    *
    * (CONECTADO A LOS PUERTOS DE ENTRADA ANALÓGICA)
    */
    int EncPort = 0; 
    int Enc2Port = 1;
    int Enc3Port = 2;
    int Enc4Port = 3; 
    //PUERTOS DE ENTRADAS ANALÓGICAS//
    double Offset1 = 0.0;
    double Offset2 = 0.0;
    double Offset3 = 0.0;
    double Offset4 = 0.0;
    //DESFACE DE LOS ENCODERS EN GRADOS//


    //CREACIÓN DEL DUTY CYCLE
    DutyCycleEncoder DutyCycleEncoder = new DutyCycleEncoder(DutyPort);
    //

    //CREACIÓN DE 4 ENCODERS ABSOLUTOS ANALOG INPUT
    AnalogInput Absencoder1 = new AnalogInput(EncPort);
    AnalogInput Absencoder2 = new AnalogInput(Enc2Port);
    AnalogInput Absencoder3 = new AnalogInput(Enc3Port);
    AnalogInput Absencoder4 = new AnalogInput(Enc4Port);
    //
    
  @Override
  public void robotInit() {

    DutyCycleEncoder.setPositionOffset(Units.degreesToRotations(DutyCycleEncOffset)); //PONE EL OFFSET DEL DUTY CICLE

  }
  /**
   * FUNCIÓN QUE REGRESA LA POSICIÓN EN GRADOS DEL ENCODER
   * <p> TAMBIÉN AJUSTA EL OFFSET DEL ENCODER ANTES DE MANDAR LA POSICIÓN
   * 
   * @param enc El encoder absoluto
   * @param inputoffset El offset del encoder absoluto
   * 
   * @return La posición del encoder en grados
   */
  public double getPosition(AnalogInput enc, double inputoffset){
    double encoderBits = enc.getValue();
    double angleEncoder = (encoderBits * 360) / 4096;
    return angleEncoder - inputoffset; 

  }

  @Override
  public void robotPeriodic() {

    //MOSTRAR TODAS LAS POSICIONES EN LA SHUFFLEBOARD PARA DEBUGGEAR


    //MOSTRAR EN LA SHUFFLEBOARD EL ENCODER DUTYCYCLE (ENCODER A LA DIO):
    double Turretposition = Units.rotationsToDegrees(DutyCycleEncoder.getDistance());

    SmartDashboard.putNumber( "Encoder torreta", Turretposition);
    //

    //MOSTRAR LOS OTROS ENCODERS ABSOLUTOS (ANALOG INPUT):
    SmartDashboard.putNumber("AbsEncoder PORT: " + EncPort, getPosition(Absencoder1, Offset1));
    SmartDashboard.putNumber("AbsEncoder PORT: " + Enc2Port, getPosition(Absencoder2, Offset2));
    SmartDashboard.putNumber("AbsEncoder PORT: " + Enc3Port, getPosition(Absencoder3, Offset3));
    SmartDashboard.putNumber("AbsEncoder PORT: " + Enc4Port, getPosition(Absencoder4, Offset4));
    //
  }


  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
