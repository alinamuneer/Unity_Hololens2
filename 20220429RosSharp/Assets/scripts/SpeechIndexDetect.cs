using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit;
using System;

namespace RosSharp.RosBridgeClient
{
    public class SpeechIndexDetect : MonoBehaviour, IMixedRealitySpeechHandler
    {
        protected EnableTeleoperationPublisher  ETPublisher;
        // Start is called before the first frame update
        void Start()
        {
            CoreServices.InputSystem.RegisterHandler<IMixedRealitySpeechHandler>(this);
            ETPublisher = gameObject.AddComponent<EnableTeleoperationPublisher>();
            ETPublisher.Topic = "enable_teleop";
        }

        public void OnSpeechKeywordRecognized(SpeechEventData eventData)
        {
            switch (eventData.Command.Keyword.ToLower())
            {
                case "stop":
                    // Debug.Log("stop all!");
                    ETPublisher.EnableIndex = Convert.ToSByte(0);
                    ETPublisher.publish = true;
                    break;
                case "stop right":
                    // Debug.Log("stop right!");
                    ETPublisher.EnableIndex = Convert.ToSByte(4);
                    ETPublisher.publish = true;
                    break;
                case "start right":
                    // Debug.Log("start right!");
                    ETPublisher.EnableIndex = Convert.ToSByte(2);
                    ETPublisher.publish = true;
                    break;
                case "stop left":
                    // Debug.Log("stop left!");
                    ETPublisher.EnableIndex = Convert.ToSByte(3);
                    ETPublisher.publish = true;
                    break;
                case "start left":
                    // Debug.Log("start left!");
                    ETPublisher.EnableIndex = Convert.ToSByte(1);
                    ETPublisher.publish = true;
                    break;
                default:
                    // Debug.Log("Unknown option");
                    ETPublisher.publish = false;
                    break;
            }
        }
    }
}


