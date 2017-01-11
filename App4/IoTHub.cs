using Microsoft.Azure.Devices.Client;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Windows.Devices.Gpio;

namespace PuppyCareApp
{
    class IoTHub
    {
        public static CancellationTokenSource _receiveTokenSource2;
        public static Task _AutoWater;
        Queue<object> _buffer;
        public static DeviceClient deviceClient;
        public static string deviceConnectionStr = "HostName=AVN-group.azure-devices.net;DeviceId=RaspberyData;SharedAccessKey=en9r0KNg4+zG15msYX/aFjtut4n0R+2fmi3iVU9GC+E=";

        /// <summary>
        /// Формирует буфер (очередь) для передачи данных в IoTHub 
        /// </summary>
        /// <param name="buffer">Объекты для передачи </param>

        public IoTHub(Queue<object> buffer)
        {
            _buffer = buffer;
        }

        /// <summary>
        /// Передает данные в IoTHub в отдельном потоке 
        /// </summary>

        public async Task Send(CancellationToken token)
        {
            deviceClient = DeviceClient.CreateFromConnectionString(deviceConnectionStr, TransportType.Http1);
            while (!token.IsCancellationRequested)//!token.IsCancellationRequested
            {
                try
                {
                    List<object> dBuffer = new List<object>();
                    lock (_buffer)
                    {
                        while (_buffer.Count > 0)
                        {
                            var d = _buffer.Dequeue();
                            dBuffer.Add(d);
                        }
                    }
                    foreach (var d in dBuffer)
                    {
                        var msg = new Message(Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(d)));
                        await deviceClient.SendEventAsync(msg);
                    }
                }
                catch (Exception exc)
                {
                    Debug.WriteLine("Sending error");
                    Debug.Write(exc.Message);
                }
                await Task.Delay(3000);
            }

        }

        /// <summary>
        /// Принимает данные из IoTHub (фоновый процесс, постоянное прослушивание на наличие новых запросов на получение)
        /// </summary>

        public async Task Receive(CancellationToken token)
        {
            deviceClient = DeviceClient.CreateFromConnectionString(deviceConnectionStr, TransportType.Http1);
            while (!token.IsCancellationRequested)//!token.IsCancellationRequested
            {
                try
                {
                    Message receivedMessage = await deviceClient.ReceiveAsync();
                    if (receivedMessage != null)
                    {
                        string Command = Encoding.ASCII.GetString(receivedMessage.GetBytes());
                        switch (Command)
                        {
                            case "1":
                                InitGpio._relePin.Write(GpioPinValue.High);
                                break;
                            case "-1":
                                InitGpio._relePin.Write(GpioPinValue.Low);
                                break;
                            default:
                                break;
                        }
                        await deviceClient.CompleteAsync(receivedMessage);
                    }
                }
                catch (Exception exc)
                {
                    Debug.Write(exc.Message);
                }
                await Task.Delay(500);
            }
        }

    }
}


