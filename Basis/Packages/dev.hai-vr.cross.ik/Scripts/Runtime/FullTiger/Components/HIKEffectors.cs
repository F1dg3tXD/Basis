// Copyright 2025 Haï~ (@vr_hai github.com/hai-vr)
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//    http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
using System.Linq;
using Unity.Mathematics;
using UnityEngine;

namespace HVR.IK.FullTiger
{
    /// Creates the end effectors that will be used by the IK solver.
    /// The end effectors placements should be modified by external modules before the IK solver runs.
    public class HIKEffectors : MonoBehaviour
    {
        public Animator animator;

        [Header("Spine")]
        public bool hipPositionMattersMore;
        public bool contortionist;
        public bool doNotPreserveHipsToNeckCurvatureLimit;
        [Range(0, 1)]
        public float improveSpineBuckling = 1f;

        [Header("Automatic Chest")]
        [Range(0, 1)]
        public float chestRotationUsesHead = 0f;

        [Header("Chest effector")]
        [Range(0, 1)]
        public float useChest;
        [Range(0, 1)]
        public float alsoUseChestToMoveNeck;

        [Header("Arm bend")]
        [Range(0, 1)]
        public float useLeftLowerArm;
        [Range(0, 1)]
        public float useRightLowerArm;

        [Header("Struggle")]
        public float legStruggleStart = HIKObjective.StruggleStart;
        public float legStruggleEnd = HIKObjective.StruggleEnd;
        public float armStruggleStart = HIKObjective.StruggleStart;
        public float armStruggleEnd = HIKObjective.StruggleEnd;

        [Header("Shoulder")]
        [Range(0, 1)]
        public float useShoulder = 0f;
        [Range(0, 1)]
        public float shoulderForwardAngleMultiplier = 1f;
        [Range(0, 1)]
        public float shoulderUpwardAngleMultiplier = 1f;

        [Header("Straddling")]
        public bool useStraddlingLeftLeg;
        public bool useStraddlingRightLeg;

        [Header("Self-parenting, Left Hand")]
        [Range(0, 1)]
        public float useSelfParentLeftHand;
        public HumanBodyBones selfParentLeftHandBone;
        public float3 selfParentLeftHandRelativePosition;
        public float3 selfParentLeftHandRelativeRotationEuler;

        [Header("Self-parenting, Right Hand")]
        [Range(0, 1)]
        public float useSelfParentRightHand;
        public HumanBodyBones selfParentRightHandBone;
        public float3 selfParentRightHandRelativePosition;
        public float3 selfParentRightHandRelativeRotationEuler;

        [Header("Environmental")]
        [Range(0, 1)]
        public float useHipsFromEnvironmental = 0f;

        [Header("Experimental (CHANGES POSITION)")]
        [Range(0, 1)]
        public float useFakeDoubleJointedKnees = 0f;

        public Vector3 hipWorldPosition;
        public Quaternion hipWorldRotation;
        public Vector3 headWorldPosition;
        public Quaternion headWorldRotation;
        public Vector3 leftHandWorldPosition;
        public Quaternion leftHandWorldRotation;
        public Vector3 rightHandWorldPosition;
        public Quaternion rightHandWorldRotation;
        public float3 leftFootWorldPosition;
        public quaternion leftFootWorldRotation;
        public float3 rightFootWorldPosition;
        public quaternion rightFootWorldRotation;
        public Vector3 chestTargetWorldPosition;
        public Quaternion chestTargetWorldRotation;
        public float3 leftLowerArmWorldPosition;
        public quaternion leftLowerArmWorldRotation;
        public float3 rightLowerArmWorldPosition;
        public quaternion rightLowerArmWorldRotation;
        public float3 groundedStraddlingLeftLegWorldPosition;
        public quaternion groundedStraddlingLeftLegWorldRotation;
        public float3 groundedStraddlingRightLegWorldPosition;
        public quaternion groundedStraddlingRightLegWorldRotation;


        public bool IsInitialized() => _isInitialized;
        private bool _isInitialized;

        public void Initalize()
        {
            _isInitialized = true;
        }
    }
}
#endif
