using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class TCPManager : MonoBehaviour
{
    public static TCPManager Instance { get; private set; }
    public Dictionary<string, string> SensorData { get; private set; }

    private TcpClient client;
    private NetworkStream stream;
    private Thread receiveThread;
    private bool isRunning = true;

    void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(this.gameObject);
            SensorData = new Dictionary<string, string>(); // Stores sensor data
        }
        else
        {
            Destroy(this.gameObject);
        }
    }

    void Start()
    {
        ConnectToESP();
    }

    private void ConnectToESP()
    {
        try
        {
            client = new TcpClient("192.168.4.1", 80); // Replace with NodeMCU IP and port
            stream = client.GetStream();

            receiveThread = new Thread(ReceiveMessages);
            receiveThread.IsBackground = true;
            receiveThread.Start();

            Debug.Log("Connected to NodeMCU.");
        }
        catch (Exception e)
        {
            Debug.LogError("Failed to connect to NodeMCU: " + e.Message);
        }
    }

    private void ReceiveMessages()
    {
        try
        {
            byte[] buffer = new byte[1024];

            while (isRunning)
            {
                if (stream != null && stream.DataAvailable)
                {
                    int bytesRead = stream.Read(buffer, 0, buffer.Length);
                    string message = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                    Debug.Log("Message from NodeMCU: " + message);

                    // Split message by newlines and process each line
                    string[] lines = message.Split(new[] { '\n' }, StringSplitOptions.RemoveEmptyEntries);
                    foreach (string line in lines)
                    {
                        ProcessMessage(line.Trim());
                    }

                    // ProcessMessage(message);
                }

                Thread.Sleep(10);
            }
        }
        catch (Exception e)
        {
            Debug.LogError("Error in ReceiveMessages: " + e.Message);
        }
    }

    private void ProcessMessage(string message)
    {
        if (!string.IsNullOrWhiteSpace(message))
        {
            string[] parts = message.Split(new char[] { ':' }, 2); // Split into two parts only
            if (parts.Length == 2)
            {
                string key = parts[0].Trim();
                string value = parts[1].Trim();

                lock (SensorData)
                {
                    if (!string.IsNullOrEmpty(key))
                    {
                        SensorData[key] = value; // Update the sensor value
                    }
               
                }
            }
              else
            {
                Debug.LogWarning($"Invalid message format: {message}"); // Log unexpected messages
            }
        }
        // foreach (var kvp in SensorData)
        // {
        //     Debug.Log($"Key: {kvp.Key}, Value: {kvp.Value}");
        // }
    }

    void OnDestroy()
    {
        isRunning = false;

        if (receiveThread != null && receiveThread.IsAlive)
        {
            receiveThread.Join();
        }

        if (stream != null)
        {
            stream.Close();
        }

        if (client != null)
        {
            client.Close();
        }

        Debug.Log("TCP Connection closed.");
    }
}
