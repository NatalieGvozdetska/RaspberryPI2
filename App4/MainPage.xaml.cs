using System;
using System.Collections.Generic;
using Windows.UI.Xaml.Controls;
using System.Threading;
using System.Threading.Tasks;
using Windows.Devices.Gpio;

namespace PuppyCareApp
{
    public sealed partial class MainPage : Page
    {
        private static Queue<object> _buffer;
        private static Task _receiveTask;
        private static Task _sendTask;
        private static Task _getSendingTask;
        public static CancellationTokenSource _receiveTokenSource;
        private static string DeviceID = "RaspberryData";

        public MainPage()
        {
            this.InitializeComponent();
            InitGpio.Init();
        }

        /// <summary>
        /// Инициализирует передачу и прием данных из IoTHub
        /// </summary>

        public static void SendReceiveStart()
        {
            _receiveTokenSource = new CancellationTokenSource();
            _buffer = new Queue<object>();
            IoTHub hub = new IoTHub(_buffer);
            _getSendingTask = FormObjectToSend(_receiveTokenSource.Token);
            _receiveTask = hub.Receive(_receiveTokenSource.Token);
            _sendTask = hub.Send(_receiveTokenSource.Token);
        }

        public static async Task FormObjectToSend(CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                DateTime date = DateTime.Now;
                JSONobj obj2send = new JSONobj();
                obj2send.temperature = Bluetooth.temperature;
                obj2send.pulse = Bluetooth.pulse;
                obj2send.latitude = Bluetooth.latitude;
                obj2send.longitude = Bluetooth.longitude;
                obj2send.datetime = date.ToString();
                obj2send.deviceID = DeviceID;
                lock (_buffer)
                {
                    _buffer.Enqueue(obj2send);
                }
                await Task.Delay(3000);
            }
        }

        /// <summary>
        /// Включает мигание сигнальной лампочки в зависимости от события
        /// </summary>
        /// <param name="pin">Указывает, какой именно лампочке мигать (номер GPIO Pin, к которому присоединена лампочка)</param>
        /// <param name="count">Количество миганий лампочки </param>

        public static async Task CheckLEDBlink(GpioPin pin, int count)
        {
            int k = 0;
            while (k < count)
            {
                pin.Write(GpioPinValue.High);
                await Task.Delay(500);
                pin.Write(GpioPinValue.Low);
                await Task.Delay(500);
                k++;
            }
        }
    }
}
