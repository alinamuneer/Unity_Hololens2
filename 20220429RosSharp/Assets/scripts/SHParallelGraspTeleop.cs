using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class SHParallelGraspTeleop : GripperTeleop
    {
        // Pr2 Gripper Command Publisher
        private Float32Publisher publisher;

        Quaternion th_proximal_init_state;
        Quaternion th_base_init_state;

        protected override void Start()
        {
            publisher = gameObject.AddComponent<Float32Publisher>();
            publisher.Topic = "rh_parallel_grasp_controller/command";

            //gripperViz = GameObject.Find("rh_viz");
            //gripperVizGoal = GameObject.Find("rh_viz_goal");
            //th_base_init_state = gripperViz.transform.Find("rh_palm/rh_thbase").localRotation;
            //th_proximal_init_state = gripperViz.transform.Find("rh_palm/rh_thbase/rh_thproximal").localRotation;
        }

        protected override void SetJogGoal(float state)
        {
            if (enableTeleop)
                publisher.Data = state;
        }

        //protected override void UpdateVisualization(GameObject gripper)
        //{
        //    float angle;
        //    Joint UnityJoint;

         //   // First finger rh_FFJ3
         //   angle = (1.47f - state * 10) * -Mathf.Rad2Deg;
        //    UnityJoint = gripper.transform.Find("rh_palm/rh_ffknuckle/rh_ffproximal").GetComponent<UnityEngine.Joint>();
        //    gripper.transform.Find("rh_palm/rh_ffknuckle/rh_ffproximal").localRotation = Quaternion.AngleAxis(angle, UnityJoint.axis);

        //    // Middle finger rh_MFJ3
        //    angle = (1.57f - state * 10) * -Mathf.Rad2Deg;
        //    UnityJoint = gripper.transform.Find("rh_palm/rh_mfknuckle/rh_mfproximal").GetComponent<UnityEngine.Joint>();
        //    gripper.transform.Find("rh_palm/rh_mfknuckle/rh_mfproximal").localRotation = Quaternion.AngleAxis(angle, UnityJoint.axis);

        //   // Thumb THJ5
        //    angle = (0.3f - state * 7) * -Mathf.Rad2Deg;
        //    UnityJoint = gripper.transform.Find("rh_palm/rh_thbase").GetComponent<UnityEngine.Joint>();
        //    gripper.transform.Find("rh_palm/rh_thbase").localRotation = th_base_init_state * Quaternion.AngleAxis(angle, UnityJoint.axis);

        //    // Thumb THJ4
        //    angle = 1.2f * -Mathf.Rad2Deg;
        //    UnityJoint = gripper.transform.Find("rh_palm/rh_thbase/rh_thproximal").GetComponent<UnityEngine.Joint>();
        //    gripper.transform.Find("rh_palm/rh_thbase/rh_thproximal").localRotation = th_proximal_init_state * Quaternion.AngleAxis(angle, UnityJoint.axis);
       // }
    }
}