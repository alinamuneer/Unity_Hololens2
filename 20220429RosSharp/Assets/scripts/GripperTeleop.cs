using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;

namespace RosSharp.RosBridgeClient
{
    public abstract class GripperTeleop : MonoBehaviour
    {
        protected bool enableTeleop = false;

        // Hand detection stuff
        public Handedness recordingHand;
        protected MixedRealityPose indexTip;
        protected MixedRealityPose thumbTip;

        // Gripper opening state
        protected float state;

        //protected GameObject gripperViz;
        //protected GameObject gripperVizGoal;

        // 1. Init publisher
        // 2. Init GameObjects
        protected abstract void Start();

        protected virtual void Update()
        {
            if (HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexTip, recordingHand, out indexTip) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbTip, recordingHand, out thumbTip))
            {

                state = Vector3.Distance(indexTip.Position, thumbTip.Position) - 0.01f;
                SetJogGoal(state);

                // UpdateVisualization(gripperViz);
                //if (enableTeleop)
                    //UpdateVisualization(gripperVizGoal);
            }
        }

        protected abstract void SetJogGoal(float state);

        //protected abstract void UpdateVisualization(GameObject gripper);

        public void StartTeleop()
        {
            enableTeleop = true;
        }

        public void StopTeleop()
        {
            enableTeleop = false;
        }
    }
}