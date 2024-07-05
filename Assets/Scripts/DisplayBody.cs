using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class DisplayBody
{
    public class DisplayBone
    {
        public Transform transform;
        public quaternion bind_rot;
        public float3 bind_pos;

        public void Bind(Transform transform)
        {
            this.transform = transform;
            bind_pos = transform.position;
            bind_rot = transform.rotation;
        }
    }

    public DisplayBone chest = new DisplayBone();
    public DisplayBone arm_top_l = new DisplayBone();
    public DisplayBone arm_bottom_l = new DisplayBone();
    public DisplayBone arm_top_r = new DisplayBone();
    public DisplayBone arm_bottom_r = new DisplayBone();
    public DisplayBone head = new DisplayBone();
    public DisplayBone belly = new DisplayBone();
    public DisplayBone pelvis = new DisplayBone();
    public DisplayBone leg_top_l = new DisplayBone();
    public DisplayBone leg_bottom_l = new DisplayBone();
    public DisplayBone leg_top_r = new DisplayBone();
    public DisplayBone leg_bottom_r = new DisplayBone();
}
