using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Door : MonoBehaviour
{
    public float interactionDistance = 3.0f;
    public GameObject intText;
    public string doorOpenAnimName = "DoorOpen";
    public string doorCloseAnimName = "DoorClose";
    public Transform player;

    private Animator doorAnim;
    private bool isPlayerInRange;
    private bool isOpen = false;

    // protected KeyCode doorKey = KeyCode.None; // Default key
    private string trigger;

    void Start()
    {
        doorAnim = GetComponent<Animator>();
        if (intText != null)
        {
            intText.SetActive(false);  // Ensure the interaction text is hidden at the start
        }
    }

    void Update()
    {
        CheckPlayerDistance();

        if (isPlayerInRange)
        {
            if (intText != null)
            {
                intText.SetActive(true);  // Show interaction text when player is in range
            }

            trigger = IRsensorcheck();      // Returns the IR sensor status from sub class function

            if (trigger == "yes") // Use the condition to toggle door
            {
                ToggleDoor();
            }
    
        }
        else
        {
            if (intText != null)
            {
                intText.SetActive(false);  // Hide interaction text when player is out of range
            }
        }
    }

    // New method for subclasses to override
    protected virtual string IRsensorcheck()
    {
        // Default: returns no . Subclasses can override this.
        return "no";
    }

    void CheckPlayerDistance()
    {
        if (player != null)
        {
            float distance = Vector3.Distance(player.position, transform.position);
            isPlayerInRange = distance <= interactionDistance;
        }
    }

    void ToggleDoor()
    {

        if (isOpen)
        {
            doorAnim.ResetTrigger("open");
            doorAnim.SetTrigger("close");
        }
        else
        {
            doorAnim.ResetTrigger("close");
            doorAnim.SetTrigger("open");
        }

        isOpen = !isOpen;  // Toggle the door state
    }
}
