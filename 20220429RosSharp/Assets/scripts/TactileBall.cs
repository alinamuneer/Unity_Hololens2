using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class TactileBall : MonoBehaviour
{
    private PR2GripperFindContactDataSubscriber tactileData;
    public GameObject Ball;
    private Renderer renderer;
    private bool colorMode = false;

    void Start()
    {
        tactileData = gameObject.AddComponent<PR2GripperFindContactDataSubscriber>();
        tactileData.Topic = "/l_gripper_sensor_controller/contact_state";

        renderer = Ball.GetComponent<Renderer>();
    }

    void Update()
    {
        float force = (float)(tactileData.message.left_fingertip_pad_force + tactileData.message.right_fingertip_pad_force) / 2;

        Color color;

        if (!colorMode)
            color = new Color(0.0f, 1.0f, 1.0f, 1.0f);
        else
        {
            if (force < 5.0f)
            {
                // From white to green
                color = new Color(1.0f - (force / 5.0f), 1.0f, 1.0f - (force / 5.0f), 1.0f);
            }
            else if (force < 7.5f)
            {
                // From green to yellow
                color = new Color((force - 5.0f) / 2.5f, 1.0f, 0.0f, 1.0f);
            }
            else if (force < 10.0f)
            {
                // From yellow to red
                color = new Color(1.0f, 1.0f - ((force - 7.5f) / 2.5f), 0.0f, 1.0f);
            }
            else
            {
                // Red
                color = new Color(1.0f, 0.0f, 0.0f, 1.0f);
            }
        }
        renderer.material.color = color;
    }

    public void EnableBall(bool enable)
    {
        colorMode = enable;
    }
}