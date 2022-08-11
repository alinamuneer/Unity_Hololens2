using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class PR2GripperTeleop : GripperTeleop
    {
        // Pr2 Gripper Command Publisher
        private Pr2GripperCommandPublisher Pr2GCPPublisher;

        protected override void Start()
        {
            Pr2GCPPublisher = gameObject.AddComponent<Pr2GripperCommandPublisher>();
            Pr2GCPPublisher.Topic = "/hololens_gripper_control/position_command";
            Pr2GCPPublisher.publish = false;

            //gripperViz = GameObject.Find("gripper_viz");
            //gripperVizGoal = GameObject.Find("gripper_viz_goal");
        }

        protected override void SetJogGoal(float state)
        {
            if (enableTeleop)
            {
                Pr2GCPPublisher.publish = true;
                Pr2GCPPublisher.MaxEffort = 30.0f;
                Pr2GCPPublisher.Position = state;
            }
            else
                Pr2GCPPublisher.publish = false;
        }

        //protected override void UpdateVisualization(GameObject gripper)
        //{
        //    float s = state * 300;
        //    gripper.transform.Find("l_gripper_l_finger_link").localRotation = Quaternion.AngleAxis(-s, Vector3.up);
        //    gripper.transform.Find("l_gripper_r_finger_link").localRotation = Quaternion.AngleAxis(s, Vector3.up);
        //    gripper.transform.Find("l_gripper_l_finger_link/l_gripper_l_finger_tip_link").localRotation = Quaternion.AngleAxis(s, Vector3.up);
        //    gripper.transform.Find("l_gripper_r_finger_link/l_gripper_r_finger_tip_link").localRotation = Quaternion.AngleAxis(-s, Vector3.up);
        //}
    }
}
