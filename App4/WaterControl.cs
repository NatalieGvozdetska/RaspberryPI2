using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Windows.Devices.Gpio;

namespace App4
{
    class WaterControl
    {
        public static int output;
        public static bool status = false;

        public static async Task WaterLevel()
        {
            while (true)
            {
                try
                {
                    var transmitBuffer = new byte[3] { 1, 0x80, 0x00 };
                    var receiveBuffer = new byte[3];
                    InitGpio._mcp3008.TransferFullDuplex(transmitBuffer, receiveBuffer);
                    //first byte returned is 0 (00000000), 
                    //second byte returned we are only interested in the last 2 bits 00000011 (mask of &3) 
                    //then shift result 8 bits to make room for the data from the 3rd byte (makes 10 bits total)
                    //third byte, need all bits, simply add it to the above result 
                    var result = ((receiveBuffer[1] & 3) << 8) + receiveBuffer[2];
                    output = result / 10;//32
                    if (output < 13)
                    {
                        status = false;
                    }
                    if (output > 60)
                    {
                        status = true;
                    }
                    await Task.Delay(1000);
                }
                catch (Exception)
                {
                    await MainPage.blink(InitGpio._CheckWater, 3);
                    await WaterLevel();
                    }
            }
        }
        public static async Task AutoWater(CancellationToken token)
        {
            while (!token.IsCancellationRequested)
                if (output < 13)
                {
                    // open cran and check every second
                    InitGpio._relePin.Write(GpioPinValue.High);
                    await Task.Delay(500);
                }
            if (output > 60)
            {
                //close cran and check every minute
                InitGpio._relePin.Write(GpioPinValue.Low);
                await Task.Delay(4000);
            }
        }

    }
} 
