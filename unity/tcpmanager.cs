using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class TCPManager : MonoBehaviour
{
    public static TCPManager Instance { get; private set; } // Singleton Instance
    public Dictionary<string, string> SensorData { get; private set; }  // Stores sensor readings

    private TcpClient client;
    private NetworkStream stream;
    private Thread receiveThread;
    private bool isRunning = false;

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
            isRunning = true;

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
        byte[] buffer = new byte[1024];

        while (isRunning)
        {
            try
            {
                if (stream != null && stream.DataAvailable)
                {
                    int bytesRead = stream.Read(buffer, 0, buffer.Length);
                    string message = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                    Debug.Log("Message from NodeMCU: " + message);
                    
                    foreach (string line in message.Split('/'))
                        ProcessMessage(line.Trim());
                }
            }
            catch (Exception e)
            {
                Debug.LogError("Receive error: " + e.Message);
            }
            
            Thread.Sleep(10);
        }
    }

    private void ProcessMessage(string message)
    {
        if (string.IsNullOrWhiteSpace(message)) return;

        string[] parts = message.Split(':', 2);
        if (parts.Length == 2)
        {
            lock (SensorData)
                SensorData[parts[0].Trim()] = parts[1].Trim();
        }
        else
        {
            Debug.LogWarning($"Invalid message format: {message}");
        }
    }

     void OnDestroy()
    {
        isRunning = false;
        receiveThread?.Join();
        stream?.Close();
        client?.Close();

        Debug.Log("TCP Connection closed.");
    }
}
