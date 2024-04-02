package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LED extends SubsystemBase{
  public static final int pwmPort = 9;
  private static final int ledLength = 50;
  private final AddressableLED led = new AddressableLED(pwmPort);
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(ledLength);
  private final double maxTotalBrightness = 1500;
  public double trackTime = 0;

  public LED() {
    led.setLength(ledLength);
    led.start();
  }

  // Run on true for flashing so the time for tracking how long to do something is known.
  public void getTime(){
    this.trackTime = Timer.getFPGATimestamp();
  }

  public Command setLedsToSolidColor(Color color){
    return runLeds(() -> setLedBufferToSolidColor(color));
  }

  public void setLedBufferToSolidColor(Color color){
    for (int ledIndex = 0; ledIndex < ledLength; ledIndex++) {
      ledBuffer.setLED(ledIndex, color);
    }
  }

  public Command setLedsToFlashAlternatingColor(Color color){
    return runLeds(() -> setLedBufferToFlashAlternatingColor(color));
  }

  public void setLedBufferToFlashAlternatingColor(Color color){
    int seconds = (int)Timer.getFPGATimestamp();
    for (int ledIndex = 0; ledIndex < ledLength; ledIndex++) {
      if (ledIndex % 2 == seconds % 2) {
        ledBuffer.setLED(ledIndex, color);
      }
    }
  }

  public void setLedBufferToFlashColor(Color color, double timeInSeconds){
    double currentTime = Timer.getFPGATimestamp();
    if(trackTime < currentTime-timeInSeconds){
      setLedBufferToSolidColor(color);
    } else {
      setLedBufferToSolidColor(Color.kBlack);
    }
  }

  public Command runLeds(Runnable action) {
    return runOnce(
      () -> {
        action.run();
        showLeds();
      })
      .ignoringDisable(true);
  }

  public void showLeds() {
    powerLimitLeds();
    led.setData(ledBuffer);
  }

  public void powerLimitLeds() {
    double totalBrightness = 0;
    for (int ledIndex = 0; ledIndex < ledLength; ledIndex++) {
      totalBrightness += 
      ledBuffer.getLED(ledIndex).red + 
      ledBuffer.getLED(ledIndex).green + 
      ledBuffer.getLED(ledIndex).blue;
    }
    double brightnessDivider = totalBrightness/maxTotalBrightness;
    if(maxTotalBrightness>1) {
      for (int ledIndex = 0; ledIndex < ledLength; ledIndex++) {
        ledBuffer.setLED(ledIndex, new Color(
          ledBuffer.getLED(ledIndex).red/brightnessDivider, 
          ledBuffer.getLED(ledIndex).green/brightnessDivider, 
          ledBuffer.getLED(ledIndex).blue/brightnessDivider));
      }
    }

  }
}
