using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestJoint : MonoBehaviour
{
    public Vector3[] jointValues;

    public Vector3[] getJoints()
    {
        return jointValues;
    }

    public Vector3 getJoint()
    {
        return jointValues[0];
    }
}