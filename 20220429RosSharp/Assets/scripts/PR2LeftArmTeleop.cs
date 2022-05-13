using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using System.Collections.Generic;
using System.Linq;

namespace RosSharp.RosBridgeClient
{
    public class PR2LeftArmTeleop : ArmTeleop
    {
        // Hand detection stuff
        private MixedRealityPose indexTip;
        private MixedRealityPose indexKnuckle;
        private MixedRealityPose indexMetacarpal;
        private MixedRealityPose thumbTip;
        private MixedRealityPose thumbProximal;
        private MixedRealityPose thumbMetacarpal;

        private List<Vector3> averageTipCenter;
        private List<Quaternion> averageFinalRotation;


        protected override void Start()
        {
            base.Start();
            // Init Jog Frame Publisher
            TSPublisher = gameObject.AddComponent<TransformStampedPublisher>();
            TSPublisher.Topic = "/left/goal_transform_stamp";
            TSPublisher.parent_frame_ID = "base_link";
            TSPublisher.child_frame_ID = "l_gripper_tool_frame";
            TSPublisher.PublishedTransform.SetParent(GameObject.Find("base_link").transform);

            currentEndEffectorTransform = GameObject.Find("l_gripper_palm_link/l_gripper_tool_frame");

            averageTipCenter = new List<Vector3>();
            averageFinalRotation = new List<Quaternion>();

            // Vizualisation stuff
           // gripperViz = GameObject.Find("gripper_viz");
            //gripperVizGoal = GameObject.Find("gripper_viz_goal");
            goalBall = GameObject.Find("GoalPoseLaBall");
        }

        protected override void Update()
        {
            if (HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexTip, recordingHand, out indexTip) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbTip, recordingHand, out thumbTip) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexKnuckle, recordingHand, out indexKnuckle) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbProximalJoint, recordingHand, out thumbProximal) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexMetacarpal, recordingHand, out indexMetacarpal) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbMetacarpalJoint, recordingHand, out thumbMetacarpal))
            {

                // Update hand transform
                UpdateHandTransform();

                // Update goal transform
                UpdateGoalTransform();

                // set Jog position and visualize mirrored gripper
                SetGoal();

                // Update Gripper vizualisation on hand
               // UpdateGripperPose();

                // visualize gripper at goal position
                UpdateGoalPose();
            }
        }

        protected override void UpdateHandTransform()
        {
            // Gripper vizualisation on hand position
            // tipCenter in World frame is used to fit visual grippers tip
            Vector3 thumbTipToIndexTip = thumbTip.Position - indexTip.Position;
            Vector3 tipCenter = new Vector3((float)(thumbTip.Position.x - thumbTipToIndexTip.x * 0.5),
                                            (float)(thumbTip.Position.y - thumbTipToIndexTip.y * 0.5),
                                            (float)(thumbTip.Position.z - thumbTipToIndexTip.z * 0.5));

            // nuckle center to align direction
            Vector3 thumbKnuckleToIndexKnuckle = thumbProximal.Position - indexKnuckle.Position;
            Vector3 knuckleCenter = new Vector3((float)(thumbProximal.Position.x - thumbKnuckleToIndexKnuckle.x * 0.5),
                                            (float)(thumbProximal.Position.y - thumbKnuckleToIndexKnuckle.y * 0.5),
                                            (float)(thumbProximal.Position.z - thumbKnuckleToIndexKnuckle.z * 0.5));

            // Gripper vizualisation on hand rotation
            // Get center between thumb and index metacarpals
            Vector3 thumbMetaToIndexMeta = thumbMetacarpal.Position - indexMetacarpal.Position;
            Vector3 MetaCenter = new Vector3((float)(thumbMetacarpal.Position.x - thumbMetaToIndexMeta.x * 0.5),
                                                (float)(thumbMetacarpal.Position.y - thumbMetaToIndexMeta.y * 0.5),
                                                (float)(thumbMetacarpal.Position.z - thumbMetaToIndexMeta.z * 0.5));

            // Fit direction Vector for gripper forword direction
            Vector3 normalOnThumbToTip = GetNormalVectorOnLine(thumbTip.Position, thumbKnuckleToIndexKnuckle, MetaCenter) * -1;

            Quaternion knuckleRotation = Quaternion.FromToRotation(Vector3.right, thumbKnuckleToIndexKnuckle);
            Quaternion metaKnuckleRotation = Quaternion.FromToRotation(Vector3.forward, normalOnThumbToTip);

            // Get rotation angle to align gripper with forward direction
            Vector3 z1 = knuckleRotation * Vector3.forward;
            Vector3 z2 = metaKnuckleRotation * Vector3.forward;
            Vector3 axis = knuckleRotation * Vector3.right;
            float angle = Vector3.SignedAngle(z1, z2, axis);

            Quaternion finalRotation = knuckleRotation * Quaternion.Euler(angle, 0, 0);

            // Average over last 5 transforms to prevent jittering
            averageTipCenter.Add(tipCenter);
            if (averageTipCenter.Count > 5)
                averageTipCenter.RemoveAt(0);
            var averagedCenter = new Vector3(
                averageTipCenter.Average(x => x.x),
                averageTipCenter.Average(x => x.y),
                averageTipCenter.Average(x => x.z));

            averageFinalRotation.Add(finalRotation);
            if (averageFinalRotation.Count > 5)
                averageFinalRotation.RemoveAt(0);

            var averageRotation = AverageQuaternion(averageFinalRotation);

            handTransform.transform.SetPositionAndRotation(averagedCenter, averageRotation);
        }

        //protected override void UpdateGripperPose()
        //{
         //   TransformParentWithChild(gripperViz.transform, gripperViz.transform.Find("l_gripper_tool_frame"), handTransform.transform.position, handTransform.transform.rotation);
        //}

        protected override void UpdateGoalPose()
        {
            goalBall.transform.SetPositionAndRotation(goalTransform.transform.position, goalTransform.transform.rotation);

            // Positioning and rotating goal gripper
            //TransformParentWithChild(gripperVizGoal.transform, gripperVizGoal.transform.Find("l_gripper_tool_frame"), goalTransform.transform.position, goalTransform.transform.rotation);
        }
    }
}