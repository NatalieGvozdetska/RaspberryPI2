using System;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Windows.Devices.Bluetooth.Rfcomm;
using Windows.Devices.Enumeration;
using Windows.Networking.Sockets;
using Windows.Storage.Streams;
using Windows.Devices.Gpio;

/// <summary>
/// Объекты класса Bluetooth могут быть использованы для приема и передачи данных
/// </summary>
/// <param name="dataReaderObject">Параметр для считывания данных</param>
/// <param name="_pairedDevices">Список устройств в паре с RPi</param>

namespace PuppyCareApp
{
    class Bluetooth
    {
        private static StreamSocket _socket;
        private static DataReader dataReaderObject;
        public static ObservableCollection<PairedDeviceInfo> _pairedDevices;
        private static RfcommDeviceService _service;
        private static CancellationTokenSource ReadCancellationTokenSource;
        public static string receivedResult;
        public static string latitude = "";
        public static string longitude = "";
        public static string temperature = "";
        public static string pulse = "";
        public static bool success = true;


        public class PairedDeviceInfo
        {
            internal PairedDeviceInfo(DeviceInformation deviceInfo)
            {
                this.DeviceInfo = deviceInfo;
                this.ID = this.DeviceInfo.Id;
                this.Name = this.DeviceInfo.Name;
            }
            public string Name { get; private set; }
            public string ID { get; private set; }
            public DeviceInformation DeviceInfo { get; private set; }
        }

        /// <summary>
        /// Инициализирует найденное в зоне действия блютуз устройство, если с ним установлена пара
        /// </summary>

        public static async void InitializeRfcommDeviceService()
        {
            try
            {
                DeviceInformationCollection DeviceInfoCollection = await DeviceInformation.FindAllAsync(RfcommDeviceService.GetDeviceSelector(RfcommServiceId.SerialPort));
                var numDevices = DeviceInfoCollection.Count();

                // By clearing the backing data, we are effectively clearing the ListBox
                _pairedDevices = new ObservableCollection<PairedDeviceInfo>();
                _pairedDevices.Clear();

                if (numDevices == 0)
                {
                    System.Diagnostics.Debug.WriteLine("InitializeRfcommDeviceService: No paired devices found.");
                }
                else
                {
                    // Found paired devices.
                    foreach (var deviceInfo in DeviceInfoCollection)
                    {
                        _pairedDevices.Add(new PairedDeviceInfo(deviceInfo));
                    }
                }
                Debug.WriteLine(_pairedDevices[0].Name);
                ConnectDevice();
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("InitializeRfcommDeviceService: " + ex.Message);
            }
        }

        async private static void ConnectDevice()
        {
            //Revision: No need to requery for Device Information as we alraedy have it:
            DeviceInformation DeviceInfo; // = await DeviceInformation.CreateFromIdAsync(this.TxtBlock_SelectedID.Text);
            PairedDeviceInfo pairedDevice = _pairedDevices[0];
            DeviceInfo = pairedDevice.DeviceInfo;
            try
            {
                while (_service == null)
                {
                    _service = await RfcommDeviceService.FromIdAsync(DeviceInfo.Id);
                }

                if (_socket != null)
                {
                    // Disposing the socket with close it and release all resources associated with the socket
                    _socket.Dispose();
                }

                _socket = new StreamSocket();
                try
                {
                    // Note: If either parameter is null or empty, the call will throw an exception
                    await _socket.ConnectAsync(_service.ConnectionHostName, _service.ConnectionServiceName);
                    success = true;
                    MainPage.SendReceiveStart();
                }
                catch (Exception ex)
                {
                    success = false;
                    ConnectDevice();
                    System.Diagnostics.Debug.WriteLine("Connect:" + ex.Message);
                }
                // If the connection was successful, the RemoteAddress field will be populated
                if (success)
                {
                    string msg = String.Format("Connected to {0}!", _socket.Information.RemoteAddress.DisplayName);
                    //MessageDialog md = new MessageDialog(msg, Title);
                    System.Diagnostics.Debug.WriteLine(msg);
                    Listen();
                    //await md.ShowAsync();
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("Overall Connect: " + ex.Message);
                _socket.Dispose();
                _socket = null;
            }
        }

        private static async void Listen()
        {
            try
            {
                ReadCancellationTokenSource = new CancellationTokenSource();
                if (_socket.InputStream != null)
                {
                    dataReaderObject = new DataReader(_socket.InputStream);
                    // keep reading the serial input
                    while (true)
                    {//read
                        await ReadAsync(ReadCancellationTokenSource.Token);
                    }
                }
            }
            catch (Exception ex)
            {
                if (ex.GetType().Name == "TaskCanceledException")
                {
                    System.Diagnostics.Debug.WriteLine("Listen: Reading task was cancelled, closing device and cleaning up");
                }
                else
                {
                    System.Diagnostics.Debug.WriteLine("Listen: " + ex.Message);
                }
            }
            finally
            {
                // Cleanup once complete
                if (dataReaderObject != null)
                {
                    dataReaderObject.DetachStream();
                    dataReaderObject = null;
                }
            }
        }

        private static async Task ReadAsync(CancellationToken cancellationToken)
        {
            Task<UInt32> loadAsyncTask;
            uint ReadBufferLength = 2048;

            // If task cancellation was requested, comply 
            cancellationToken.ThrowIfCancellationRequested();

            // Set InputStreamOptions to complete the asynchronous read operation when one or more bytes is available 
            dataReaderObject.InputStreamOptions = InputStreamOptions.Partial;

            // Create a task object to wait for data on the serialPort.InputStream 
            loadAsyncTask = dataReaderObject.LoadAsync(ReadBufferLength).AsTask(cancellationToken);

            // Launch the task and wait 
            UInt32 bytesRead = await loadAsyncTask;
            if (bytesRead > 0)
            {
                try
                {
                    string recvdtxt = dataReaderObject.ReadString(bytesRead);
                    receivedResult += recvdtxt;
                    string[] lastSymb;

                    if (recvdtxt.EndsWith("\r\n"))
                    {
                        //System.Diagnostics.Debug.WriteLine(result); 
                        lastSymb = receivedResult.Split(',');
                        latitude = lastSymb[0];
                        longitude = lastSymb[1];
                        temperature = lastSymb[2];
                        pulse = lastSymb[3].Split(';')[0]; ;
                        receivedResult = "";
                    }
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine("ReadAsync: " + ex.Message);
                }

            }
            else
            {
                await MainPage.CheckLEDBlink(InitGpio._BluetoothPin, 3);
                if (MainPage._receiveTokenSource != null)
                {
                    MainPage._receiveTokenSource.Cancel();
                    InitGpio._BluetoothPin.Write(GpioPinValue.Low);
                }
                ConnectDevice();
            }
        }
    }
}


