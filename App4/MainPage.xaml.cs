using System;
using System.Threading;
using System.Threading.Tasks;
using Windows.Devices.Gpio;
using System.Collections.Generic;

namespace PuppyCareApp
{
    public sealed partial class MainPage
    {
        private static Queue<object> _buffer;
        private static Task _receiveTask;
        private static Task _sendTask;
        private static Task _getSendingTask;
        public static CancellationTokenSource _receiveTokenSource;
        private static string DeviceID = "RaspberryData";
        private bool onof = false;
        private int i = 0;


        public MainPage()
        {
            //Initiailizing pins
            InitGpio.Init();
            InitializeComponent();
        }

        /// <summary>
        /// Инициализирует передачу и прием данных из IoTHub
        /// </summary>

        public static void sendreceive()
        {
            _receiveTokenSource = new CancellationTokenSource();
            _buffer = new Queue<object>();
            HubMsg hub = new HubMsg(_buffer);
             _getSendingTask = GetDate(_receiveTokenSource.Token);
            _receiveTask = hub.Receive(_receiveTokenSource.Token);
            _sendTask = hub.Sending(_receiveTokenSource.Token);
        }

        public static async Task GetDate(CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                DateTime date = DateTime.Now;
                Data d = new Data();
                d.valueTemp = Bluetooth.temperature;
                d.Lat = Bluetooth.lat;
                d.Lon = Bluetooth.lon; 
                d.valueDateTime = date.ToString();
                d.valueID = DeviceID;
                lock (_buffer)
                    {
                        _buffer.Enqueue(d);
                    }
                await Task.Delay(3000);
            }
        }

        /// <summary>
        /// Включает мигание сигнальной лампочки в зависимости от события
        /// </summary>
        /// <param name="pin">Указывает, какой именно лампочке мигать </param>
        /// <param name="count">Количество миганий лампочки </param>
        
        public static async Task blink(GpioPin pin,int count)
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