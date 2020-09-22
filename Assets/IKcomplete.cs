#if UNITY_EDITOR
using UnityEditor;
#endif
using System;
using System.CodeDom;
using System.Collections.Specialized;
using System.Diagnostics;
using System.Text;
using UnityEngine;

namespace DitzelGames.FastIK
{
    /// <summary>
    /// Fabrik IK Solver
    /// </summary>
    public class IKcomplete : MonoBehaviour
    {
        /// <summary>
        /// Chain length of bones
        /// </summary>
        public int ChainLength = 3;

        /// <summary>
        /// Target the chain should bent to
        /// </summary>
        public Transform Target;

        /// <summary>
        /// Solver iterations per update
        /// </summary>
        [Header("Solver Parameters")]
        public int Iterations = 1;

        /// <summary>
        /// Distance when the solver stops
        /// </summary>
        public float Delta = 0.001f;

        /// <summary>
        /// Strength of going back to the start position.
        /// </summary>
        [Range(0, 1)]
        public float SnapBackStrength = 1f;

        protected float[] BonesLength; //Target to Origin
        protected float CompleteLength;
        protected Transform[] Bones;
        protected Vector3[] Positions;
        protected Vector3[] PositionsOld;
        protected Vector3[] StartDirectionSucc;
        protected Quaternion[] StartRotationBone;
        protected Quaternion StartRotationTarget;
        protected Transform Root;

        public Vector3[] constraintPos, constraintNeg;

        //vector storing a bool for each bone in the chain, if it is 1 it's affected by a first-type constraint(handled by the constrainDirectionXandY) or second-type(handled by the constrainDirectionXorYandZ)
        public bool[] boneConstraintType;

        //here are stored the quaternion for each bone which give the correct computed rotation and orientation for the scene
        protected Quaternion[] boneSceneRotation;

        //the axis rotation to send to the robot for achieve computed direction and orientation
        protected Vector3[] robotBoneRotations;

        public Vector3 upVec, rightVec;

        public Vector3 initialUp, initialDown;

        public Vector3 initialRight, initialLeft;

        public float zConstraintPos, zConstraintNeg;

        public float zConstraintPosBound, zConstraintNegBound;

        public float deltaRotation;

        public float xAspect, yAspect;

        public float scalar;

        public float xBound, yBound;




        // Start is called before the first frame update
        void Awake()
        {
            Init();


        }

        void Init()
        {
            //initial array
            Bones = new Transform[ChainLength + 1];
            Positions = new Vector3[ChainLength + 1];
            BonesLength = new float[ChainLength];
            StartDirectionSucc = new Vector3[ChainLength + 1];
            StartRotationBone = new Quaternion[ChainLength + 1];

            boneSceneRotation = new Quaternion[ChainLength];

            robotBoneRotations = new Vector3[ChainLength + 1];

            //find root
            Root = transform;
            for (var i = 0; i <= ChainLength; i++)
            {
                if (Root == null)
                    throw new UnityException("The chain value is longer than the ancestor chain!");
                Root = Root.parent;
            }

            //init target
            if (Target == null)
            {
                Target = new GameObject(gameObject.name + " Target").transform;
                SetPositionRootSpace(Target, GetPositionRootSpace(transform));
            }
            StartRotationTarget = GetRotationRootSpace(Target);


            //init data
            var current = transform;
            CompleteLength = 0;
            for (var i = Bones.Length - 1; i >= 0; i--)
            {
                Bones[i] = current;
                StartRotationBone[i] = GetRotationRootSpace(current);


                if (i == Bones.Length - 1)
                {
                    //leaf
                    StartDirectionSucc[i] = GetPositionRootSpace(Target) - GetPositionRootSpace(current);
                }
                else
                {
                    //mid bone
                    StartDirectionSucc[i] = GetPositionRootSpace(Bones[i + 1]) - GetPositionRootSpace(current);
                    BonesLength[i] = StartDirectionSucc[i].magnitude;
                    CompleteLength += BonesLength[i];
                }

                current = current.parent;
            }
        }

        // Update is called once per frame
        void LateUpdate()
        {
            ResolveIK();
        }

        private void ResolveIK()
        {
            if (Target == null)
                return;

            if (BonesLength.Length != ChainLength)
                Init();

            //Fabrik

            //  root
            //  (bone0) (bonelen 0) (bone1) (bonelen 1) (bone2)...
            //   x--------------------x--------------------x---...

            //get position
            for (int i = 0; i < Bones.Length; i++)
                Positions[i] = GetPositionRootSpace(Bones[i]);
            PositionsOld = Positions;

            var targetPosition = GetPositionRootSpace(Target);
            var targetRotation = GetRotationRootSpace(Target);//Target

            /*for (var i = 0; i <= ChainLength - 1; i++)
            {
                boneSceneRotation[i] = StartRotationBone[i];
            }*/

            //1st is possible to reach?
            if ((targetPosition - GetPositionRootSpace(Bones[0])).sqrMagnitude >= CompleteLength * CompleteLength)
            {
                //just strech it
                var direction = (targetPosition - Positions[0]).normalized;

                //set everything after root
                for (int i = 1; i < Positions.Length; i++)
                {
                    Positions[i] = Positions[i - 1] + direction * BonesLength[i - 1];

                    //if first bone must compare direction to initial direction of bone
                    if (i == 1)
                    {
                        //check what type of bone it is
                        if (boneConstraintType[i - 1] == true)
                            Positions[i] = Positions[i - 1] + constrainDirectionXandY(direction * BonesLength[i - 1], StartDirectionSucc[0], i);
                        else
                            Positions[i] = Positions[i - 1] + constrainDirectionXorYandZ(direction * BonesLength[i - 1], StartDirectionSucc[0], i);
                    }

                    //else to previous bone direction
                    else
                    {
                        //check what type of bone it is
                        if (boneConstraintType[i - 1] == true)
                            Positions[i] = Positions[i - 1] + constrainDirectionXandY(direction * BonesLength[i - 1], (Positions[i - 1] - Positions[i - 2]), i);
                        else
                            Positions[i] = Positions[i - 1] + constrainDirectionXorYandZ(direction * BonesLength[i - 1], (Positions[i - 1] - Positions[i - 2]), i);
                    }
                }

            }
            else
            {
                for (int i = 0; i < Positions.Length - 1; i++)
                    Positions[i + 1] = Vector3.Lerp(Positions[i + 1], Positions[i] + StartDirectionSucc[i], SnapBackStrength);

                for (int iteration = 0; iteration < Iterations; iteration++)
                {
                    //back
                    for (int i = Positions.Length - 1; i > 0; i--)
                    {
                        if (i == Positions.Length - 1)
                            Positions[i] = targetPosition; //set it to target
                        else
                            Positions[i] = Positions[i + 1] + (Positions[i] - Positions[i + 1]).normalized * BonesLength[i]; //set in line on distance
                    }

                    //forward
                    for (int i = 1; i < Positions.Length; i++)
                    {
                        Positions[i] = Positions[i - 1] + (Positions[i] - Positions[i - 1]).normalized * BonesLength[i - 1];

                        //if first bone must compare direction to initial direction of bone
                        if (i == 1)
                        {
                            //check what type of bone it is
                            if (boneConstraintType[i - 1] == true)
                                Positions[i] = Positions[i - 1] + constrainDirectionXandY((Positions[i] - Positions[i - 1]).normalized * BonesLength[i - 1], StartDirectionSucc[0], i);
                            else
                                Positions[i] = Positions[i - 1] + constrainDirectionXorYandZ((Positions[i] - Positions[i - 1]).normalized * BonesLength[i - 1], StartDirectionSucc[0], i);
                        }
                        //else to previous bone direction
                        else
                        {
                            //check what type of bone it is
                            if (boneConstraintType[i - 1] == true)
                                Positions[i] = Positions[i - 1] + constrainDirectionXandY((Positions[i] - Positions[i - 1]).normalized * BonesLength[i - 1], (Positions[i - 1] - Positions[i - 2]), i);
                            else
                                Positions[i] = Positions[i - 1] + constrainDirectionXorYandZ((Positions[i] - Positions[i - 1]).normalized * BonesLength[i - 1], (Positions[i - 1] - Positions[i - 2]), i);
                        }

                    }
                    //close enough?
                    //if ((Positions[Positions.Length - 1] - targetPosition).sqrMagnitude < Delta * Delta)
                    //break;
                }


            }
            //compute wrist rotation based on elbow Z rotation
            //UnityEngine.Debug.Log(normalizeAngle(Target.localEulerAngles.z));
            //UnityEngine.Debug.Log(robotBoneRotations[1].z);

            robotBoneRotations[2] = new Vector3(0, 0, constrainWristOnZ(robotBoneRotations[1].z, Target.localEulerAngles.z));
            //UnityEngine.Debug.Log(robotBoneRotations[2].z);

            Quaternion tmp;
            //set position & rotation

            //UnityEngine.Debug.Log(Target.localEulerAngles.z);

            for (int i = 0; i < Positions.Length; i++)
            {
                if (i == Positions.Length - 1)
                {
                    //UnityEngine.Debug.Log(robotBoneRotations[1].z);
                    //Bones[i].rotation = boneSceneRotation[i - 1];/*SetRotationRootSpace(Bones[i], Quaternion.Inverse(targetRotation) * StartRotationTarget * Quaternion.Inverse(StartRotationBone[i]));*/
                }
                else
                {
                    tmp = Quaternion.FromToRotation(StartDirectionSucc[i], Positions[i + 1] - Positions[i]);



                    if (boneConstraintType[i])
                        SetRotationRootSpace(Bones[i], Quaternion.Euler(tmp.eulerAngles.x, tmp.eulerAngles.y, 0f) * Quaternion.Inverse(StartRotationBone[i]));
                    else
                        SetRotationRootSpace(Bones[i], Quaternion.Euler(tmp.eulerAngles.x, tmp.eulerAngles.y, robotBoneRotations[i - 1].z) * Quaternion.Inverse(StartRotationBone[i]));

                }
                SetPositionRootSpace(Bones[i], Positions[i]);
            }


        }

        private Vector3 GetPositionRootSpace(Transform current)
        {
            if (Root == null)
                return current.position;
            else
                return Quaternion.Inverse(Root.rotation) * (current.position - Root.position);
        }

        private void SetPositionRootSpace(Transform current, Vector3 position)
        {
            if (Root == null)
                current.position = position;
            else
                current.position = Root.rotation * position + Root.position;
        }

        private Quaternion GetRotationRootSpace(Transform current)
        {
            //inverse(after) * before => rot: before -> after
            if (Root == null)
                return current.rotation;
            else
                return Quaternion.Inverse(current.rotation) * Root.rotation;
        }

        private void SetRotationRootSpace(Transform current, Quaternion rotation)
        {
            if (Root == null)
                current.rotation = rotation;
            else
                current.rotation = Root.rotation * rotation;
        }

        //function used to constrain Zora Shoulder joint
        private Vector3 constrainDirectionXandY(Vector3 calcDirection, Vector3 initialDirection, int i)
        {


            Quaternion rotation = Quaternion.LookRotation(initialDirection);

            Quaternion rotationNoZ = Quaternion.Euler(rotation.eulerAngles.x, rotation.eulerAngles.y, 0f);


            initialUp = rotationNoZ * Vector3.up;

            initialDown = -initialUp;

            initialRight = rotationNoZ * Vector3.right;

            initialLeft = -initialRight;


            //i take the axis closer to the line
            if ((initialUp - calcDirection).magnitude < ((initialDown - calcDirection).magnitude))
                upVec = initialUp;
            else
                upVec = initialDown;

            if ((initialRight - calcDirection).magnitude < ((initialLeft - calcDirection).magnitude))
                rightVec = initialRight;
            else
                rightVec = initialLeft;


            //used to check if projection must be flipped
            float scalar = Vector3.Dot(calcDirection, initialDirection) / initialDirection.magnitude;

            //projected tip of calculated vector on initial direction(centre of cone)
            Vector3 projPoint = Positions[0] + Vector3.Project(calcDirection, initialDirection.normalized);

            //vector from previous joint to projPoint
            Vector3 projDirection = projPoint - Positions[0];

            //angle > 90 degrees, must flip projection vector
            if (scalar < 0)
            {
                projDirection = -projDirection;
            }

            //vector from tip of projection to calculated direction tip
            Vector3 adjust = calcDirection - projDirection;

            //our fix, to review
            float xAspect = Vector3.Dot(adjust, rightVec);

            if (rightVec != initialRight) xAspect = -xAspect;

            float yAspect = Vector3.Dot(adjust, upVec);

            if (upVec != initialUp) yAspect = -yAspect;

            //elipse widths
            float left = -(projDirection.magnitude * Mathf.Tan(constraintNeg[i - 1].x * Mathf.Deg2Rad));
            float right = projDirection.magnitude * Mathf.Tan(constraintPos[i - 1].x * Mathf.Deg2Rad);
            float up = projDirection.magnitude * Mathf.Tan(constraintPos[i - 1].y * Mathf.Deg2Rad);
            float down = -(projDirection.magnitude * Mathf.Tan(constraintNeg[i - 1].y * Mathf.Deg2Rad));


            float xBound, yBound;

            //find quadrant
            if (xAspect >= 0) xBound = right;
            else xBound = left;

            if (yAspect >= 0) yBound = up;
            else yBound = down;


            Vector3 finalDirection = calcDirection;

            float ellipse = ((xAspect * xAspect) / (xBound * xBound)) + ((yAspect * yAspect) / (yBound * yBound));

            //if outside violets constrains, must correct
            if (ellipse > 1f || scalar < 0)
            {
                float a = Mathf.Atan2(yAspect, xAspect);
                float x = xBound * Mathf.Cos(a);
                float y = yBound * Mathf.Sin(a);

                finalDirection = (projDirection + rightVec * x + upVec * y).normalized * calcDirection.magnitude;
            }

            rotation = Quaternion.LookRotation(finalDirection);

            rotationNoZ = Quaternion.Euler(rotation.eulerAngles.x, rotation.eulerAngles.y, 0f);

            float robotx = rotation.eulerAngles.x;

            float roboty = rotation.eulerAngles.y;

            //temporary fix for adjusting joint angle values in Zora world
            if (robotx > 91) robotx = robotx - 360f;

            if (roboty > 77f) roboty = roboty - 360f;

            robotBoneRotations[i - 1] = new Vector3(robotx, roboty, 0);

            boneSceneRotation[i - 1] = rotationNoZ;

            return finalDirection;
        }

        //function used to constrain the Elbow joint of Zora, which can move in the Y and Z axis(XorY refers to a more generic purpose, but only Y is actually used for Zora since the elbow is constrained on Y axis e Z axis)
        private Vector3 constrainDirectionXorYandZ(Vector3 calcDirection, Vector3 initialDirection, int i)
        {

            //get up and right vector of initial direction vector(the bone original direction in default pose or previous bone one)
            Quaternion rotation = Quaternion.LookRotation(initialDirection);

            Quaternion rotationNoZ = Quaternion.Euler(rotation.eulerAngles.x, rotation.eulerAngles.y, 0f);

            upVec = rotationNoZ * Vector3.up;

            rightVec = rotationNoZ * Vector3.right;



            //used to check if projection must be flipped
            scalar = Vector3.Dot(calcDirection, initialDirection) / initialDirection.magnitude;

            //projected tip of calculated direction on initial direction(centre of cone)
            Vector3 projPoint = Positions[0] + Vector3.Project(calcDirection, initialDirection.normalized);

            //vector from previous joint to projPoint
            Vector3 projDirection = projPoint - Positions[0];

            //angle > 90 degrees, must flip projection vector
            if (scalar < 0)
            {
                projDirection = -projDirection;
            }

            //vector from tip of projection to calculated direction tip
            Vector3 adjust = calcDirection - projDirection;


            //xDistance from center
            xAspect = Vector3.Dot(adjust, rightVec);

            //yDistance from center
            yAspect = Vector3.Dot(adjust, upVec);


            //determine  which constraint between x and y is beeing used and if positive or negative
            float xOrYconstraint;

            Vector3 constraintAxisXorY;

            //this variable represents the offset between the axis of the x or y constraint and the angle 0 of a normal goniometric circumference on which calculations will be done
            float angleOffset = 0f;

            //the elbow y constraint is the left arm one
            if (constraintPos[i - 1].x > 0)
            {
                xOrYconstraint = constraintPos[i - 1].x;
                constraintAxisXorY = rightVec;
                angleOffset = 0f;

            }
            //the elbow y constraint is the right arm one
            else if (constraintNeg[i - 1].x > 0)
            {
                xOrYconstraint = constraintNeg[i - 1].x;
                constraintAxisXorY = -rightVec;
                angleOffset = 180.0f;
            }
            //not used for zora
            else if (constraintPos[i - 1].y > 0)
            {
                xOrYconstraint = constraintPos[i - 1].y;
                constraintAxisXorY = upVec;
                angleOffset = 90f;
            }
            //not used for zora
            else
            {
                xOrYconstraint = constraintNeg[i - 1].y;
                constraintAxisXorY = -upVec;
                angleOffset = 270.0f;
            }

            //they are equal, used to build a circle with radius equal to projected x or y constraint angle, that's where the bone can go without keeping in mind the z constraint(for now)
            float right = projDirection.magnitude * Mathf.Tan(xOrYconstraint * Mathf.Deg2Rad);
            float up = projDirection.magnitude * Mathf.Tan(xOrYconstraint * Mathf.Deg2Rad);

            xBound = right;

            yBound = up;


            Vector3 finalDirection = calcDirection;

            float circle = ((xAspect * xAspect) / (xBound * xBound)) + ((yAspect * yAspect) / (yBound * yBound));

            //angle made by point x,y in the circle in degrees
            float angleDeg = normalizeAngle(Mathf.Atan2(yAspect, xAspect) * Mathf.Rad2Deg);

            //if outside circle, must correct point to be on the circle line
            if (circle > 1f || scalar < 0)
            {
                float a = Mathf.Atan2(yAspect, xAspect);

                xAspect = xBound * Mathf.Cos(a);
                yAspect = yBound * Mathf.Sin(a);
            }

            zConstraintPos = constraintPos[i - 1].z;

            zConstraintNeg = -constraintNeg[i - 1].z;

            //must check if z angle violates z positive and negative constraints

            //obtain z positive constraint bound normalized between 0 and 360
            zConstraintPosBound = normalizeAngle(360f + (angleOffset - zConstraintPos));

            //obtain z negative constraint bound normalized between 0 and 360
            zConstraintNegBound = normalizeAngle(angleOffset + -(zConstraintNeg));

            float deltaRotation = adjustAngleBetweenBounds(zConstraintPosBound, zConstraintNegBound, angleDeg);

            angleDeg = angleDeg + deltaRotation;

            //if deltaRotation is different from 0 it means the actual angle must be adjusted to stay in bounds
            if (deltaRotation != 0f)
            {
                //rotate point (xAspect,yAspect) of deltaRotation in the circle(around origin point 0,0) to adjust it using formula in https://academo.org/demos/rotation-about-point/
                xAspect = xAspect * Mathf.Cos(deltaRotation * Mathf.Deg2Rad) - yAspect * Mathf.Sin(deltaRotation * Mathf.Deg2Rad);
                yAspect = yAspect * Mathf.Cos(deltaRotation * Mathf.Deg2Rad) + xAspect * Mathf.Sin(deltaRotation * Mathf.Deg2Rad);
            }

            finalDirection = (projDirection + rightVec * xAspect + upVec * yAspect).normalized * calcDirection.magnitude;

            //angle of x or y constraint axis in the final direction
            float xOryAngle = Vector3.Angle(finalDirection, initialDirection);


            float upTestRotation = angleDeg - angleOffset;

            //get a quaternion loyal to the rotation of this bone given the constraints

            Quaternion boneRotation = Quaternion.Euler(0.0f, 0.0f, 0.0f);

            if (xAspect >= 0.0001f || yAspect >= 0.0001f || xAspect <= -0.0001f || yAspect <= -0.0001f)
            {

                if (constraintAxisXorY == rightVec)//used in the left arm
                {
                    //upTest.rotation = Quaternion.Euler(0f, xOryAngle, 0f);
                    //store the rotation of zora angles of the wrist, y is the Roll and z is the Yaw
                    robotBoneRotations[i - 1] = new Vector3(0, xOryAngle, -getSignedRotation(angleDeg));


                }
                else if (constraintAxisXorY == upVec)//not used in zora constraint
                {
                    //upTest.rotation = Quaternion.Euler(xOryAngle, 0f , 0f);
                    robotBoneRotations[i - 1] = new Vector3(xOryAngle, 0, angleOffset - angleDeg);
                }
                else if (constraintAxisXorY == -rightVec)//used in the right arm
                {
                    //upTest.rotation = Quaternion.Euler(0f, -xOryAngle, 0f);
                    //store the rotation of zora angles of the wrist, y is the Roll and z is the Yaw
                    robotBoneRotations[i - 1] = new Vector3(0, -xOryAngle, angleOffset - angleDeg);
                }
                else
                {
                    //not used in zora constraint
                    //upTest.rotation = Quaternion.Euler(-xOryAngle, 0f, 0f);
                    robotBoneRotations[i - 1] = new Vector3(-xOryAngle, 0, angleOffset - angleDeg);
                }



                //boneRotation = boneSceneRotation[i - 2] * upTest.rotation * Quaternion.Euler(0f, 0f, angleDeg - angleOffset);

                //upTest.Rotate(new Vector3(0f, 0f, (/*angleDeg - angleOffset*/ angleDeg - angleOffset)), Space.World);

                //boneSceneRotation[i - 1] = boneRotation;
            }
            else
            {
                //boneSceneRotation[i - 1] = boneSceneRotation[i - 2];
                robotBoneRotations[i - 1] = new Vector3(0f, -xOryAngle, 0f);
                //upTest.rotation = boneSceneRotation[i - 2];
            }



            return finalDirection;
        }

        //returns the Z rotation which Zora Wrist joint can perform to equal the target one(which is the user hand tracked)
        private float constrainWristOnZ(float elbowZrotation, float targetZrotation)
        {

            //calculations are made based on Zora joints 
            float finalRotation = 0f;

            targetZrotation = normalizeAngle(targetZrotation);

            if (targetZrotation == 360f) targetZrotation = 0f;

            if (elbowZrotation > 0)
            {
                finalRotation = ((360f - elbowZrotation) - targetZrotation);
            }
            else if (elbowZrotation < 0)
            {
                finalRotation = -(elbowZrotation) - targetZrotation;
            }
            else
            {
                if (targetZrotation >= 180f) finalRotation = 360f - targetZrotation;

                else if (targetZrotation < 180f) finalRotation = -targetZrotation;
            }

            return finalRotation;
        }

        //normalize angle making it between 0 and 360 degrees
        private float normalizeAngle(float angle)
        {
            float normAngle = angle;
            if (normAngle < 0) normAngle = normAngle + 360.0f;
            if (normAngle >= 360f) normAngle = normAngle - 360.0f;
            return normAngle;
        }

        //return 0 if angle is ok, else the delta it must be rotated to remain inside legal zone defined by bounds
        private float adjustAngleBetweenBounds(float posBound, float negBound, float angle)
        {

            float deltaAngle = 0f;

            if (posBound < negBound)
            {
                if (angle < posBound) deltaAngle = posBound - angle;
                else if (angle > negBound) deltaAngle = -(angle - negBound);
            }
            else if (posBound > negBound)
            {
                if (angle < posBound && angle > negBound)
                {
                    if ((posBound - angle) < (angle - negBound)) deltaAngle = posBound - angle;
                    else deltaAngle = -(angle - negBound);
                }
            }

            return deltaAngle;
        }

        //use to get the angles of Zora joints
        public Vector3[] getRobotJoints()
        {
            return robotBoneRotations;
        }

        //convert rotation value between 0 and 180 degrees and 0 and -180 degrees, used to obtain rotation of left wrist of zora in zora world
        public float getSignedRotation(float rotation)
        {
            if (rotation > 180f) rotation = -(360f - rotation);

            return rotation;
        }


        void OnDrawGizmos()
        {
#if UNITY_EDITOR
            var current = this.transform;
            for (int i = 0; i < ChainLength && current != null && current.parent != null; i++)
            {
                var scale = Vector3.Distance(current.position, current.parent.position) * 0.1f;
                Handles.matrix = Matrix4x4.TRS(current.position, Quaternion.FromToRotation(Vector3.up, current.parent.position - current.position), new Vector3(scale, Vector3.Distance(current.parent.position, current.position), scale));
                Handles.color = Color.green;
                Handles.DrawWireCube(Vector3.up * 0.5f, Vector3.one);
                current = current.parent;
            }
#endif
        }
    }
}