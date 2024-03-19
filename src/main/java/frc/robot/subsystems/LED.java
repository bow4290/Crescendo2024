package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LED extends SubsystemBase{
  public static final int pwmPort = 9;
  private static final int ledLength = 50;
  private AddressableLED led = new AddressableLED(pwmPort);
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(ledLength);

  public LED() {
    led.setLength(ledLength);
    led.start();
  }

  public Command showLeds() {
    return runOnce(() -> {
      setLedBufferToSolidColor(Color.kRed);
      led.setData(ledBuffer);
    });
  }

  public void setLedBufferToSolidColor(Color color){
    for (var ledIndex = 0; ledIndex < ledLength; ledIndex++) {
      ledBuffer.setLED(ledIndex, color);
    }
  }
}
