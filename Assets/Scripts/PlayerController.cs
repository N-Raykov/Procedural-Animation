using ImGuiNET;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;

public class PlayerController : MonoBehaviour
{
    public GameObject display_gibbon; // Character mesh and bone transforms

    DisplayBody display_body = new DisplayBody();

    //Particle simulation system
    Verlet.System complete = new Verlet.System();

    // Initial point positions for IK
    float3[] arm_ik = new float3[3];
    float3[] leg_ik = new float3[3];

    class MovementSystem
    {
        public float3 target_com;
        public float3[] limb_targets = new float3[4];
        public Verlet.System simple_rig = new Verlet.System();
        public float body_compress_amount = 0.0f;
    }

    MovementSystem walk = new MovementSystem();
    MovementSystem swing = new MovementSystem();
    MovementSystem jump = new MovementSystem();
    MovementSystem display = new MovementSystem();

    DebugInfo debug_info = new DebugInfo();

    // Simple character particle information
    float3 simple_pos;
    float3 simple_vel = float3.zero;

    // Time marker in animation sequences (so cycles can speed up and slow down without losing continuity)
    float swing_time = 0f;
    float walk_time = 0f;

    bool in_air; // Are we currently jumping?
    float in_air_amount = 0.0f; // Could be used to transition into and out of jumping, even though we just use 0 or 1 right now
    float3 jump_com_offset; // Use to preserve COM continuity when starting jumps, so it doesn't just warp to the simple_pos position
    float jump_time; // At what Time.time did jump start?
    float3 jump_point; // Where jump started (at feet)
    float predicted_land_time; // At what Time.time do we expect to land?
    float3 predicted_land_point; // Where will we land?
    float3 look_target; // For head look IK

    float on_branch = 1f; // Is 1 if running on branch, 0 if swinging underneath it
    bool wants_to_swing = false; // Whether we are transitioning to or from swinging
    float body_compress_amount = 0.0f; // Used to shorten the distance between neck and hips if necessary, e.g. during quadruped gallop
    float skate_amount = 0.0f; // Used to interpolate into and out of sliding state

    // Various parameters that were being used to tune an animation
    float base_walk_height = 0.7f;
    float tilt_offset = 0.81f;
    float gallop_offset = 0.55f; // For biped gallop
    float quad_gallop_offset = 0.25f; // For quadruped gallop
    float gallop_stride = 1.0f;
    float gallop_stride_height = 0.2f;
    float gallop_hip_rotate = -1.3f;
    float gallop_height_offset = 0.6f;
    float gallop_height = 0.012f;
    float gallop_height_base = 0.8f;
    float gallop_lean = 1.5f;
    float gallop_arm_stride_height = 0.4f;
    float gallop_arm_stride = 0.4f;
    float quad_amount = 0.0f;
    float gallop_amount = 0.0f;
    float quad_gallop_body_compress_offset = 0.4f;
    float quad_gallop_body_compress_amount = 0.15f;

    void Start()
    {
        // Starting point
        simple_pos = display_gibbon.transform.position;
        simple_pos[1] = 0f;
        simple_pos[2] = 0f;

        // Init hand positions
        for (int i = 0; i < 4; ++i)
        {
            display.limb_targets[i] = simple_pos;
            walk.limb_targets[i] = simple_pos;
            swing.limb_targets[i] = simple_pos;
        }

        // Get transforms of each skeleton point
        var root = GameObject.Find("points").transform;
        var neck = root.Find("neck");
        var stomach = root.Find("stomach");
        var pelvis = root.Find("pelvis");
        var groin = root.Find("groin");
        var head = root.Find("head");
        var shoulder = root.Find("shoulder");
        var elbow = root.Find("elbow");
        var grip = root.Find("grip");
        var hip = root.Find("hip");
        var knee = root.Find("knee");
        var foot = root.Find("foot");

        // Set up bind poses for each bone
        display_body.head.Bind(display_gibbon.transform.Find("DEF-head"));
        display_body.chest.Bind(display_gibbon.transform.Find("DEF-chest"));
        display_body.belly.Bind(display_gibbon.transform.Find("DEF-belly"));
        display_body.pelvis.Bind(display_gibbon.transform.Find("DEF-pelvis"));
        display_body.arm_top_l.Bind(display_gibbon.transform.Find("DEF-upper_arm_L"));
        display_body.arm_bottom_l.Bind(display_gibbon.transform.Find("DEF-forearm_L"));
        display_body.arm_top_r.Bind(display_gibbon.transform.Find("DEF-upper_arm_R"));
        display_body.arm_bottom_r.Bind(display_gibbon.transform.Find("DEF-forearm_R"));
        display_body.leg_top_l.Bind(display_gibbon.transform.Find("DEF-thigh_L"));
        display_body.leg_bottom_l.Bind(display_gibbon.transform.Find("DEF-shin_L"));
        display_body.leg_top_r.Bind(display_gibbon.transform.Find("DEF-thigh_R"));
        display_body.leg_bottom_r.Bind(display_gibbon.transform.Find("DEF-shin_R"));

        // Adjust elbow to match arm transform
        elbow.position = display_body.arm_bottom_r.transform.position;

        // Set up initial IK poses (just used to get bone lengths later)
        arm_ik[0] = shoulder.position;
        arm_ik[1] = elbow.position;
        arm_ik[2] = grip.position;

        leg_ik[0] = hip.position;
        leg_ik[1] = display_body.leg_bottom_r.transform.position;
        leg_ik[2] = foot.position;

        float measured_arm_length = Vector3.Distance(shoulder.position, elbow.position) + Vector3.Distance(elbow.position, grip.position);

        // Set up movement system particles and bones
        for (int i = 0; i < 4; ++i)
        {
            Verlet.System new_simple_rig;
            switch (i)
            {
                case 0: new_simple_rig = display.simple_rig; break;
                case 1: new_simple_rig = walk.simple_rig; break;
                case 2: new_simple_rig = jump.simple_rig; break;
                default: new_simple_rig = swing.simple_rig; break;
            }

            new_simple_rig.AddPoint(shoulder.position, "shoulder_r");
            new_simple_rig.AddPoint(grip.position, "hand_r");
            new_simple_rig.AddPoint((shoulder.position + Vector3.right * (neck.position[0] - shoulder.position[0]) * 2f), "shoulder_l");
            new_simple_rig.AddPoint((grip.position + Vector3.right * (neck.position[0] - grip.position[0]) * 2f), "hand_l");
            new_simple_rig.AddPoint(new float3(neck.position[0], hip.position[1], neck.position[2]), "body");
            new_simple_rig.points[0].mass = 2f;
            new_simple_rig.points[2].mass = 2f;
            new_simple_rig.points[4].mass = 4f;

            new_simple_rig.AddBone("arm_r", 0, 1);
            new_simple_rig.bones[new_simple_rig.bones.Count - 1].length[1] = measured_arm_length;
            new_simple_rig.bones[new_simple_rig.bones.Count - 1].length[0] *= 0.4f; // Allow arm to flex
            new_simple_rig.AddBone("arm_l", 2, 3);
            new_simple_rig.bones[new_simple_rig.bones.Count - 1].length[1] = measured_arm_length;
            new_simple_rig.bones[new_simple_rig.bones.Count - 1].length[0] *= 0.4f;
            new_simple_rig.AddBone("tri_top", 0, 2);
            new_simple_rig.AddBone("tri_r", 0, 4);
            new_simple_rig.AddBone("tri_l", 2, 4);
        }

        // Set up full-body IK particles and bones
        complete.AddPoint(shoulder.position, "shoulder_r");
        complete.AddPoint(grip.position, "hand_r");
        complete.AddPoint((shoulder.position + Vector3.right * (neck.position[0] - shoulder.position[0]) * 2f), "shoulder_l");
        complete.AddPoint((grip.position + Vector3.right * (neck.position[0] - grip.position[0]) * 2f), "hand_l");
        complete.AddPoint(new float3(neck.position[0], hip.position[1], neck.position[2]), "body");
        complete.AddPoint(head.position, "head");
        complete.AddPoint(neck.position, "neck");
        complete.AddPoint(stomach.position, "stomach"); // 7
        complete.AddPoint(pelvis.position, "hip"); // 8
        complete.AddPoint(groin.position, "groin");
        complete.AddPoint(hip.position, "hip_r");
        complete.AddPoint(foot.position, "foot_r");
        complete.AddPoint(hip.position + Vector3.right * (neck.position[0] - hip.position[0]) * 2f, "hip_l");
        complete.AddPoint(foot.position + Vector3.right * (neck.position[0] - foot.position[0]) * 2f, "foot_l");

        complete.AddBone("arm_r", 0, 1);
        complete.bones[complete.bones.Count - 1].length[1] = measured_arm_length;
        complete.bones[complete.bones.Count - 1].length[0] *= 0.4f;
        complete.AddBone("arm_l", 2, 3);
        complete.bones[complete.bones.Count - 1].length[1] = measured_arm_length;
        complete.bones[complete.bones.Count - 1].length[0] *= 0.4f;
        complete.AddBone("head", 5, 6);
        complete.AddBone("chest", 6, 7);
        complete.AddBone("belly", 7, 8);
        complete.AddBone("pelvis", 8, 9);
        complete.AddBone("leg_r", 10, 11);
        complete.bones[complete.bones.Count - 1].length[0] *= 0.4f;
        complete.AddBone("leg_l", 12, 13);
        complete.bones[complete.bones.Count - 1].length[0] *= 0.4f;

        // Delete visible points so we don't see it when playing game
        Destroy(root.gameObject);
    }

    // Use law of cosines to find angles of triangle
    static float GetAngleGivenSides(float a, float b, float c)
    {
        var top = (c * c - a * a - b * b);
        var divisor = (-2 * a * b);
        if (divisor == 0f)
        {
            return 0f;
        }
        return math.acos(math.clamp(top / divisor, -1f, 1f));
    }

    // Solve two bone IK problems
    static void ApplyTwoBoneIK(int start_id,
                               int end_id,
                               float3 forward,
                               float3[] ik,
                               DisplayBody.DisplayBone top,
                               DisplayBody.DisplayBone bottom,
                               List<Verlet.Point> points,
                               float3 old_axis,
                               float3 axis)
    {
        var start = points[start_id];
        var end = points[end_id];

        // Get sides of triangle formed by upper and lower limb
        float dist_a = math.distance(ik[0], ik[1]);
        float dist_b = math.distance(ik[1], ik[2]);
        float dist_c = math.distance(start.pos, end.pos);
        float old_dist_c = math.distance(ik[0], ik[2]);

        // Get angles of triangle
        var old_hinge_angle = GetAngleGivenSides(dist_a, dist_b, old_dist_c);
        var hinge_angle = GetAngleGivenSides(dist_a, dist_b, dist_c);
        var old_base_angle = GetAngleGivenSides(old_dist_c, dist_a, dist_b);
        var base_angle = GetAngleGivenSides(dist_c, dist_a, dist_b);

        // Apply rotation of entire arm (shoulder->hand)
        var base_rotation = Quaternion.LookRotation(end.pos - start.pos, forward) *
                            Quaternion.Inverse(Quaternion.LookRotation(end.bind_pos - start.bind_pos, Vector3.forward));
        // Apply additional rotation from IK
        base_rotation = Quaternion.AngleAxis(base_angle * Mathf.Rad2Deg, axis) * base_rotation *
                        Quaternion.Inverse(Quaternion.AngleAxis(old_base_angle * Mathf.Rad2Deg, old_axis));

        // Apply base and hinge rotations to actual display bones
        top.transform.position = top.bind_pos + (start.pos - start.bind_pos);
        top.transform.rotation = base_rotation * top.bind_rot;

        bottom.transform.position = top.transform.position + top.transform.rotation * Quaternion.Inverse(top.bind_rot) * (bottom.bind_pos - top.bind_pos);
        bottom.transform.rotation = Quaternion.AngleAxis(hinge_angle * Mathf.Rad2Deg, axis) * base_rotation *
                                    Quaternion.Inverse(Quaternion.AngleAxis(old_hinge_angle * Mathf.Rad2Deg, old_axis)) * bottom.bind_rot;
    }

    // Calculate bone transform that matches orientation of top and bottom points, and looks in the character "forward" direction
    void ApplyBound(DisplayBody.DisplayBone part, float3 forward, float3 bind_forward, int start, int end)
    {
        // Get midpoint and "up" direction (from start to end point)
        var up = math.normalize(complete.points[end].pos - complete.points[start].pos);
        var bind_up = math.normalize(complete.points[end].bind_pos - complete.points[start].bind_pos);
        var mid = (complete.points[end].pos + complete.points[start].pos) / 2.0f;
        var bind_mid = (complete.points[end].bind_pos + complete.points[start].bind_pos) / 2.0f;

        // Apply rotations
        var rotation = Quaternion.LookRotation(up, forward) *
                       Quaternion.Inverse(Quaternion.LookRotation(bind_up, bind_forward));
        part.transform.rotation = rotation * part.bind_rot;
        part.transform.position = mid + (float3)(rotation * (part.bind_pos - bind_mid));
    }

    static void DrawSystem(MovementSystem system, Color color)
    {
        system.simple_rig.DrawBones(color);
        for (int i = 2; i < 4; ++i)
        {
            DebugDraw.Sphere(system.limb_targets[i], color, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            // Start jump
            simple_vel[1] = 5.0f;
            if (!in_air && on_branch == 0.0f)
            {
                simple_vel[1] += 2.0f;
            }
            in_air = true;

            // Copy display rig pose from previous frame to jump rig
            for (int i = 0; i < jump.simple_rig.points.Count; ++i)
            {
                jump.simple_rig.points[i].pos = display.simple_rig.points[i].pos;
                jump.simple_rig.points[i].old_pos = math.lerp(display.simple_rig.points[i].old_pos, display.simple_rig.points[i].pos, 0.75f); // Adjust for different timestep
            }
            for (int i = 0; i < 2; ++i)
            {
                jump.simple_rig.bones[i].length[1] = display.simple_rig.bones[i].length[1];
            }

            // Initial trajectory info
            jump_time = Time.time;
            jump_point = (display.limb_targets[2] + display.limb_targets[3]) * 0.5f;
            predicted_land_time = jump_time + 5.0f;

            // Adjust COM
            float total_mass = 0f;
            var com = float3.zero;
            for (int i = 0; i < display.simple_rig.points.Count; ++i)
            {
                com += display.simple_rig.points[i].pos * display.simple_rig.points[i].mass;
                total_mass += display.simple_rig.points[i].mass;
            }
            com /= total_mass;
            jump_com_offset = com - simple_pos;
        }

        { // Use "arms" rig to drive full body IK rig
            var points = display.simple_rig.points;

            // Calculate midpoint and orientation of body triangle
            var bind_mid = (points[0].bind_pos + points[2].bind_pos + points[4].bind_pos) / 3.0f;
            var mid = (points[0].pos + points[2].pos + points[4].pos) / 3.0f;
            var forward = math.normalize(math.cross(points[0].pos - points[2].pos, points[0].pos - points[4].pos));
            var bind_forward = math.normalize(math.cross(points[0].bind_pos - points[2].bind_pos, points[0].bind_pos - points[4].bind_pos));
            var up = math.normalize((points[0].pos + points[2].pos) / 2.0f - points[4].pos);
            var bind_up = math.normalize((points[0].bind_pos + points[2].bind_pos) / 2.0f - points[4].bind_pos);

            // Copy hand and shoulder positions from simple rig
            for (int i = 0; i < 4; ++i)
            {
                complete.points[i].pos = points[i].pos;
                complete.points[i].pinned = true;
            }

            var body_rotation = math.mul(quaternion.LookRotation(forward, up),
                                          math.inverse(quaternion.LookRotation(bind_forward, bind_up)));

            // Set up spine, head and leg positions based on body rotation
            for (int i = 5; i < 14; ++i)
            {
                complete.points[i].pos = mid + math.mul(body_rotation, (complete.points[i].bind_pos - bind_mid));
                complete.points[i].pinned = true;
            }

            // Apply body compression
            complete.points[7].pinned = false;
            complete.points[8].pinned = false;
            var old_hip = complete.points[9].pos;
            for (int i = 7; i <= 9; ++i)
            {
                complete.points[i].pos = math.lerp(complete.points[i].pos, complete.points[6].pos, body_compress_amount);
            }
            complete.points[7].pos -= forward * body_compress_amount * 0.2f;
            complete.points[8].pos -= forward * body_compress_amount * 0.2f;

            for (int i = 10; i < 14; ++i)
            {
                complete.points[i].pos += complete.points[9].pos - old_hip;
            }

            // Move feet to foot targets
            for (int i = 0; i < 2; ++i)
            {
                complete.points[11 + i * 2].pos = display.limb_targets[2 + i];
            }

            // Enforce bone length constraints
            for (int i = 0; i < 2; ++i)
            {
                complete.EnforceDistanceConstraints();
            }
        }

        { // Apply full body IK rig to visual deformation bones
            var points = complete.points;

            // Get torso orientation and position
            var bind_mid = (points[0].bind_pos + points[2].bind_pos + points[9].bind_pos) / 3.0f;
            var mid = (points[0].pos + points[2].pos + points[9].pos) / 3.0f;
            var forward = -math.normalize(math.cross(points[0].pos - points[2].pos, points[0].pos - points[9].pos));
            var bind_forward = -math.normalize(math.cross(points[0].bind_pos - points[2].bind_pos, points[0].bind_pos - points[9].bind_pos));
            var up = math.normalize((points[0].pos + points[2].pos) / 2.0f - points[9].pos);
            var bind_up = math.normalize((points[0].bind_pos + points[2].bind_pos) / 2.0f - points[9].bind_pos);

            // Apply core bones
            ApplyBound(display_body.head, forward, bind_forward, 5, 6);
            ApplyBound(display_body.chest, forward, bind_forward, 6, 7);
            ApplyBound(display_body.belly, forward, bind_forward, 7, 8);
            ApplyBound(display_body.pelvis, forward, bind_forward, 8, 9);

            // Arm IK
            for (int i = 0; i < 2; ++i)
            {
                var top = display_body.arm_top_r;
                var bottom = display_body.arm_bottom_r;
                if (i == 1)
                {
                    top = display_body.arm_top_l;
                    bottom = display_body.arm_bottom_l;
                }

                int start_id = i * 2;
                int end_id = i * 2 + 1;
                var start = points[start_id];
                var end = points[end_id];

                // Adjust elbow target position
                float ik_driver = math.max(on_branch, in_air_amount);
                var ik_forward_amount = -ik_driver * 0.8f;
                var ik_up_amount = 0.1f + ik_driver * 0.5f;
                var elbow_point = ((points[2].pos + points[0].pos) * 0.5f + up * ik_up_amount + forward * ik_forward_amount);
                var bind_elbow_point = ((points[2].bind_pos + points[0].bind_pos) * 0.5f + bind_up * ik_up_amount + bind_forward * ik_forward_amount);

                if (debug_info.draw_elbow_ik_target)
                {
                    DebugDraw.Line((start.pos + end.pos) * 0.5f, elbow_point, Color.red, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
                    DebugDraw.Sphere(elbow_point, Color.red, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
                }

                var old_axis = math.normalize(math.cross((end.bind_pos + start.bind_pos) * 0.5f - bind_elbow_point, start.bind_pos - end.bind_pos));
                var axis = math.normalize(math.cross((end.pos + start.pos) * 0.5f - elbow_point, start.pos - end.pos));

                ApplyTwoBoneIK(start_id, end_id, forward, arm_ik, top, bottom, complete.points, old_axis, axis);

                if (debug_info.draw_ik_final)
                {
                    DebugDraw.Line(points[start_id].pos, bottom.transform.position, Color.white, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
                    DebugDraw.Line(points[end_id].pos, bottom.transform.position, Color.white, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
                }
            }

            // Leg IK
            for (int i = 0; i < 2; ++i)
            {
                var top = display_body.leg_top_r;
                var bottom = display_body.leg_bottom_r;
                if (i == 1)
                {
                    top = display_body.leg_top_l;
                    bottom = display_body.leg_bottom_l;
                }

                int start = i * 2 + 10;
                int end = i * 2 + 1 + 10;

                var leg_dir = points[end].pos - points[start].pos;

                // Get knee direction
                var leg_dir_flat = math.normalize(new float2(math.dot(leg_dir, forward), math.dot(leg_dir, up)));
                var leg_forward = leg_dir_flat[0] * up + leg_dir_flat[1] * -forward;

                // Get base whole-leg rotation
                var bind_rotation = Quaternion.LookRotation(points[end].bind_pos - points[start].bind_pos, Vector3.forward);
                var rotation = Quaternion.LookRotation(leg_dir, leg_forward) * bind_rotation;

                // Get knee bend axis
                var old_axis = bind_rotation * Vector3.right;
                var axis = rotation * Vector3.right;

                ApplyTwoBoneIK(start, end, leg_forward, leg_ik, top, bottom, complete.points, old_axis, axis);

                if (debug_info.draw_ik_final)
                {
                    DebugDraw.Line(points[start].pos, bottom.transform.position, Color.white, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
                    DebugDraw.Line(points[end].pos, bottom.transform.position, Color.white, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
                }
            }

            // Head look            
            // head_look_y: 50 = max look down, -70 = max look up
            // head_look_x: -90 to 90

            // Get head target in head transform space
            var target = math.normalize(display_body.head.transform.InverseTransformPoint(look_target));
            // Using sin here is not correct (should be asin or something), but looks ok so keeping it for now
            var head_look_y = math.sin(target.x) * Mathf.Rad2Deg;
            // Flatten look direction to solve other rotation axis
            var temp = target;
            temp.x = 0.0f;
            temp = math.normalize(temp);
            var head_look_x = -math.sin(temp.y) * Mathf.Rad2Deg;

            // Apply head transform
            display_body.head.transform.rotation = display_body.head.transform.rotation * Quaternion.Euler(head_look_x, 0f, 0f) * Quaternion.Euler(0f, head_look_y, 0f);
            if (head_look_y > 0.0f)
            {
                display_body.head.transform.position = display_body.head.transform.position + (Vector3)((display_body.head.transform.right) * head_look_y * -0.001f);
            }

            if (debug_info.draw_head_look)
            {
                DebugDraw.Sphere(look_target, Color.red, Vector3.one * 0.1f, Quaternion.identity, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
                DebugDraw.Line(display_body.head.transform.position, look_target, Color.red, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
            }
        }

        // Debug draw skeleton
        if (debug_info.draw_walk_rig && in_air_amount < 1.0f && on_branch > 0.0f) { DrawSystem(walk, Color.red); }
        if (debug_info.draw_swing_rig && in_air_amount < 1.0f && on_branch < 1.0f) { DrawSystem(swing, Color.cyan); }
        if (debug_info.draw_jump_rig && in_air_amount > 0.0f) { DrawSystem(jump, Color.green); }
        if (debug_info.draw_display_simple_rig) { DrawSystem(display, Color.white); }
        if (debug_info.draw_display_complete_rig) { complete.DrawBones(Color.white); }
        if (debug_info.draw_ik_final)
        {
            for (int i = 2; i < complete.bones.Count - 2; ++i)
            {
                DebugDraw.Line(complete.points[complete.bones[i].points[0]].pos, complete.points[complete.bones[i].points[1]].pos, Color.white, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
            }
            DebugDraw.Line(complete.points[0].pos, complete.points[2].pos, Color.white, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
            DebugDraw.Line(complete.points[10].pos, complete.points[12].pos, Color.white, DebugDraw.Lifetime.OneFrame, DebugDraw.Type.Xray);
        }
        display_gibbon.SetActive(debug_info.draw_gibbon);
    }
}
