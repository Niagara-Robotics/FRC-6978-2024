package frc.robot.Tasks;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.fasterxml.jackson.core.util.ByteArrayBuilder;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Mode;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Illumination {
    
    SPI controllerPort;

    public Illumination() {
        controllerPort = new SPI(SPI.Port.kOnboardCS0);
        controllerPort.setChipSelectActiveLow();
        controllerPort.setClockRate(4000000);
        controllerPort.setMode(Mode.kMode0);

        setLength((byte)0, (byte)24);
        setLength((byte)1, (byte)24);
        setLength((byte)2, (byte)24);
        setLength((byte)3, (byte)24);


        setStatic((byte)1, 0, 255, 255);
        //setStatic((byte)0, 0, 255, 255);
        setBreathing((byte)0, (short)3000, 0, 80, 0);
        setBreathing((byte)1, (short)3000, 0, 80, 0);
        //setBreathing((byte)2, (short)6000, 0, 100, 0);
        //setBreathing((byte)3, (short)1000, 0, 100, 0);

        Color8Bit idleColor = new Color8Bit(120, 0, 0);

        //setMarquee(1, (short)800, (short)800, 12, 1, 12, true, true, idleColor, new Color8Bit(0,0,0));

    }

    public void setLength(byte slot, byte length) {
        byte data[] = new byte[32];

        data[0] = (byte)0xf4;
        data[1] = slot; //slot
        data[2] = 1; //cmd: set len
        data[3] = length;

        controllerPort.write(data, 32);
    }

    public void setStatic(byte slot, int red, int green, int blue) {
        byte data[] = new byte[32];

        data[0] = (byte)0xf4;
        data[1] = slot; //slot
        data[2] = 2; //cmd: set effect
        data[3] = 1; //effect 1: static
        data[4] = 3; //effect params length

        data[5] = (byte)red;
        data[6] = (byte)green;
        data[7] = (byte)blue;
            
        controllerPort.write(data, 32);
    }

    public void setBreathing(int slot, short period, int red, int green, int blue) {
        ByteBuffer dataBuffer = ByteBuffer.allocate(32);
        dataBuffer.order(ByteOrder.LITTLE_ENDIAN);

        dataBuffer.put((byte)0xf4);
        dataBuffer.put((byte)slot); //slot
        dataBuffer.put((byte)2); //cmd: set effect
        dataBuffer.put((byte)3); //effect 3: breathing
        dataBuffer.put((byte)5); //effect params length

        dataBuffer.putShort(period);

        dataBuffer.put((byte)red);
        dataBuffer.put((byte)green);
        dataBuffer.put((byte)blue);

        controllerPort.write(dataBuffer, 32);
    }

    public void setMarquee(int slot, short period, short time, int length, int offset, int segment, boolean count_inv, boolean colour_flip, Color8Bit foreground, Color8Bit background) {
        ByteBuffer dataBuffer = ByteBuffer.allocate(32);
        dataBuffer.order(ByteOrder.LITTLE_ENDIAN);

        dataBuffer.put((byte)0xf4);
        dataBuffer.put((byte)slot); //slot
        dataBuffer.put((byte)2); //cmd: set effect
        dataBuffer.put((byte)2); //effect 2: marquee
        dataBuffer.put((byte)14); //effect params length

        dataBuffer.putShort(period);
        dataBuffer.putShort(time);

        dataBuffer.put((byte)length);
        dataBuffer.put((byte)offset);
        dataBuffer.put((byte)segment);
        dataBuffer.put((byte)background.red);
        dataBuffer.put((byte)background.green);
        dataBuffer.put((byte)background.blue);
        dataBuffer.put((byte)foreground.red);
        dataBuffer.put((byte)foreground.green);
        dataBuffer.put((byte)foreground.blue);
        
        byte inv = 0;
        if(count_inv) inv += 1;
        if(colour_flip) inv += 2;
        dataBuffer.put(inv);


        controllerPort.write(dataBuffer, 32);
    }
}
