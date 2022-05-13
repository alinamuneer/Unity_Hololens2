using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;

namespace RosSharp.RosBridgeClient
{
    public class PR2RightArmTeleop : ArmTeleop
    {
        private MixedRealityPose wrist;

        protected override void Start()
        {
            base.Start();
            // Init Jog Frame Publisher
            TSPublisher = gameObject.AddComponent<TransformStampedPublisher>();
            TSPublisher.Topic = "/right/transform_stamp";
            TSPublisher.parent_frame_ID = "base_link";
            TSPublisher.child_frame_ID = "right_arm";
            TSPublisher.PublishedTransform.SetParent(GameObject.Find("base_link").transform);

            currentEndEffectorTransform = GameObject.Find("rh_wrist/rh_palm");


            // Vizualisation stuff
            gripperViz = GameObject.Find("rh_viz");
            gripperVizGoal = GameObject.Find("rh_viz_goal");
            goalBall = GameObject.Find("GoalPoseRaBall");
        }

        protected override void Update()
        {
            if (HandJointUtils.TryGetJointPose(TrackedHandJoint.Wrist, recordingHand, out wrist))
            {
                // Update hand transform
                UpdateHandTransform();

                // Update goal transform
                UpdateGoalTransform();

                // set Jog position and visualize mirrored gripper
                SetGoal();

                // Update visualization on hand
                UpdateGripperPose();

                // visualize gripper at goal position
                UpdateGoalPose();
            }
        }

        protected override void UpdateHandTransform()
        {
            handTransform.transform.SetPositionAndRotation(wrist.Position, wrist.Rotation);
            handTransform.transform.Rotate(0, -90, -90);
        }

        protected override void UpdateGripperPose()
        {
            gripperViz.transform.SetPositionAndRotation(handTransform.transform.position, handTransform.transform.rotation);
        }


        protected override void UpdateGoalPose()
        {
            goalBall.transform.SetPositionAndRotation(TSPublisher.PublishedTransform.position, TSPublisher.PublishedTransform.rotation);

            gripperVizGoal.transform.SetPositionAndRotation(TSPublisher.PublishedTransform.position, TSPublisher.PublishedTransform.rotation);
        }
    }
}