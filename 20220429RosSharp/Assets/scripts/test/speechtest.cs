using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit;


public class speechtest : MonoBehaviour, IMixedRealitySpeechHandler
{
    // Start is called before the first frame update
    void Start()
    {
        CoreServices.InputSystem.RegisterHandler<IMixedRealitySpeechHandler>(this);
    }

    public void OnSpeechKeywordRecognized(SpeechEventData eventData)
    {

        switch (eventData.Command.Keyword.ToLower())
        {
            case "stop":
                Debug.Log("i love you!");
                break;
            default:
                Debug.Log("Unknown option");
                break;
        }
    }
}
