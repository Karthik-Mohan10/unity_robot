using UnityEngine;
using TMPro;

public class TemperatureDisplay : MonoBehaviour
{
    public float temperature = 25.0f; // Default temperature value
    private TextMeshPro textMeshPro;

    void Start()
    {
        // Get the TextMeshPro component attached to this GameObject
        textMeshPro = GetComponent<TextMeshPro>();

        // Initialize the display with the default temperature
        UpdateTemperatureDisplay();
    }

    void Update()
    {
        // For demonstration, you can update the temperature here
        // For now, it just keeps showing the default value
        // Students can change this logic to update the temperature from the ESP32
        // Fetch the temperature from the TCPManager
        if (TCPManager.Instance != null)
        {
            string tempValue;

            // Access the shared SensorData safely
            lock (TCPManager.Instance.SensorData)
            {
                TCPManager.Instance.SensorData.TryGetValue("TEMP", out tempValue);
            }

            if (!string.IsNullOrEmpty(tempValue) && float.TryParse(tempValue, out float temp))
            {
                //Debug.Log("temp value: " + temp);
                temperature = temp; // Update the temperature value
            }
        }

        // Update the display with the current temperature value
        UpdateTemperatureDisplay();
  
    }

    public void UpdateTemperatureDisplay()
    {
        // Check if the TextMeshPro component is assigned before updating the text
        if (textMeshPro != null)
        {
            // Update the text with the current temperature value
            textMeshPro.text = temperature.ToString("F2") + " °C";
        }
    }
}
