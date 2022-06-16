using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;

namespace RosSharp.RosBridgeClient
{
    public class PR2GripperTeleop : MonoBehaviour
    {
        protected bool enabledTeleop = false;
        private Pr2GripperCommandPublisher Pr2GCPPublisher;

        // Hand detection stuff
        public Handedness recordingHand;
        protected MixedRealityPose indexTip;
        protected MixedRealityPose thumbTip;

        // Gripper opening state
        protected float state;

        // protected GameObject gripperViz;
        // protected GameObject gripperVizGoal;

        // 1. Init publisher
        // 2. Init GameObjects
        private void Start()
        {
            Pr2GCPPublisher = gameObject.AddComponent<Pr2GripperCommandPublisher>();
            Pr2GCPPublisher.Topic = "/hololens_gripper_control/position_command";
            //Pr2GCPPublisher.publish = true;

            // gripperViz = GameObject.Find("gripper_viz");
            // gripperVizGoal = GameObject.Find("gripper_viz_goal");
        }

        protected virtual void Update()
        {
            if (HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexTip, recordingHand, out indexTip) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbTip, recordingHand, out thumbTip))
            {
                if (!enabledTeleop)
                    return;
                else
                    Pr2GCPPublisher.publish = true;
                state = Vector3.Distance(indexTip.Position, thumbTip.Position) - 0.01f;
                SetJogGoal(state);

                // UpdateVisualization(gripperViz);
                //  if (enableTeleop)
                //      UpdateVisualization(gripperVizGoal);
            }
            else
            { Pr2GCPPublisher.publish = false; }
        }

        protected void SetJogGoal(float state)
        {
            if (enabledTeleop)
            {
                Pr2GCPPublisher.MaxEffort = 30.0f;
                Pr2GCPPublisher.Position = state;
            }
        }

        //protected void UpdateVisualization(GameObject gripper)
       // {
        //    float s = state * 300;
         //   gripper.transform.Find("l_gripper_l_finger_link").localRotation = Quaternion.AngleAxis(-s, Vector3.up);
         //   gripper.transform.Find("l_gripper_r_finger_link").localRotation = Quaternion.AngleAxis(s, Vector3.up);
         //   gripper.transform.Find("l_gripper_l_finger_link/l_gripper_l_finger_tip_link").localRotation = Quaternion.AngleAxis(s, Vector3.up);
         //   gripper.transform.Find("l_gripper_r_finger_link/l_gripper_r_finger_tip_link").localRotation = Quaternion.AngleAxis(-s, Vector3.up);
        //}

    public void StartTeleop()
        {
            enabledTeleop = true;
        }

        public void StopTeleop()
        {
            enabledTeleop = false;
        }
    }
}
