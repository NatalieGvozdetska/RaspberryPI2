//Class for initialize GPIOs and get parameter "value" for metod PulseIn

using System;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.Gpio;
using Windows.Devices.SerialCommunication;
using Windows.Devices.Spi;

namespace App4
{
    class InitGpio
    {
        //Initialize all variable
        public static int Rele_PIN = 26;// rele on of
        public static int HubState_PIN = 16;//Green pin
        private static int BUTTON1 = 5;//button
        public static int SPI_PIN = 19;//ok spi or not
        public static int CheckWater = 13;//ok or not check water level
        public static GpioPin _relePin;
        public static GpioPin _HubStatePin;
        public static GpioPin _Button1;
        public static GpioPin _SPIPin;
        public static GpioPin _CheckWater;
        public static SpiDevice _mcp3008;
        public static Task _Water;
        public static Task _Spi;


        //Method for initialize
        public static void Init()
        {
            var gpio = GpioController.GetDefault();
            _Button1 = gpio.OpenPin(BUTTON1);
            _relePin = gpio.OpenPin(Rele_PIN);
            _HubStatePin = gpio.OpenPin(HubState_PIN);
            _SPIPin = gpio.OpenPin(SPI_PIN);
            _CheckWater = gpio.OpenPin(CheckWater);
            if (_Button1.IsDriveModeSupported(GpioPinDriveMode.InputPullUp))
                _Button1.SetDriveMode(GpioPinDriveMode.InputPullUp);
            else
                _Button1.SetDriveMode(GpioPinDriveMode.Input);
            _relePin.SetDriveMode(GpioPinDriveMode.Output);
            _HubStatePin.SetDriveMode(GpioPinDriveMode.Output);
            _SPIPin.SetDriveMode(GpioPinDriveMode.Output);
            _CheckWater.SetDriveMode(GpioPinDriveMode.Output);
            _relePin.Write(GpioPinValue.Low);//Low or High
            _HubStatePin.Write(GpioPinValue.Low);
            _SPIPin.Write(GpioPinValue.High);
            _CheckWater.Write(GpioPinValue.Low);
            _Spi = SPIConnect();
            Bluetooth.InitializeRfcommDeviceService();

        }

        private static async Task SPIConnect()  //need to call in every methods for analog sensors
        {
            //using SPI0 on the Pi
            var spiSettings = new SpiConnectionSettings(0);//for spi bus index 0
            spiSettings.ClockFrequency = 3600000; //3.6 MHz
            spiSettings.Mode = SpiMode.Mode0;

            string spiQuery = SpiDevice.GetDeviceSelector("SPI0");
            //using Windows.Devices.Enumeration;
            var deviceInfo = await DeviceInformation.FindAllAsync(spiQuery);
            if (deviceInfo != null && deviceInfo.Count > 0)
            {
                _mcp3008 = await SpiDevice.FromIdAsync(deviceInfo[0].Id, spiSettings);
                _SPIPin.Write(GpioPinValue.Low);
                _Water = WaterControl.WaterLevel();

            }
            else
            {
                await MainPage.blink(InitGpio._SPIPin, 5);
                await SPIConnect();
            }

        }
    }
}
