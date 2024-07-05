using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ImGuiNET;

class DebugInfo
{
    public bool draw_walk_rig = false;
    public bool draw_ik_final = false;
    public bool draw_swing_rig = false;
    public bool draw_jump_rig = false;
    public bool draw_display_simple_rig = false;
    public bool draw_display_complete_rig = false;
    public bool draw_gibbon = true;
    public bool draw_elbow_ik_target = false;
    public bool draw_com_line = false;
    public bool draw_simple_point = false;
    int draw_layer = 0;
    string[] layers = new string[] {
            "Skinned",
            "IK",
            "Rig",
            "Simple Rig",
            "Source Rigs",
            "Particle"
        };
    int force_gait = 0;
    string[] gaits = new string[] {
            "Normal",
            "Skate",
            "Biped Run",
            "Biped Gallop",
            "Quadruped Gallop"
        };
    public bool force_skate = false;
    public bool force_run = false;
    public bool force_gallop = false;
    public bool force_quad = false;
    public bool draw_hand_pull = false;
    public bool draw_trajectory = false;
    public bool draw_head_look = false;
    public bool draw_smoothing = false;
    public List<DebugDraw.DebugDrawLine> com_lines = new List<DebugDraw.DebugDrawLine>();

    public void DrawWindow()
    {
        if (ImGui.Begin("Debug Visualization"))
        {
            if (ImGui.Combo("Draw", ref draw_layer, layers, layers.Length))
            {
                draw_gibbon = (draw_layer == 0);
                draw_ik_final = (draw_layer == 1);
                draw_display_complete_rig = (draw_layer == 2);
                draw_display_simple_rig = (draw_layer == 3);
                draw_walk_rig = (draw_layer == 4);
                draw_swing_rig = (draw_layer == 4);
                draw_jump_rig = (draw_layer == 4);
                draw_simple_point = (draw_layer == 5);
            }
            if (ImGui.Combo("Gait", ref force_gait, gaits, gaits.Length))
            {
                force_skate = (force_gait == 1);
                force_run = (force_gait == 2);
                force_gallop = (force_gait == 3);
                force_quad = (force_gait == 4);
            }
            if (ImGui.Checkbox("Draw COM line", ref draw_com_line))
            {
                if (!draw_com_line)
                {
                    foreach (var line in com_lines)
                    {
                        DebugDraw.Remove(line);
                    }
                    com_lines.Clear();
                }
            }
            ImGui.Checkbox("Draw path smoothing", ref draw_smoothing);
            ImGui.Checkbox("Draw hand pull", ref draw_hand_pull);
            ImGui.Checkbox("Draw jump trajectory", ref draw_trajectory);
            ImGui.Checkbox("Draw head look", ref draw_head_look);
            ImGui.Checkbox("Draw elbow IK target", ref draw_elbow_ik_target);

            bool slow_motion = (Time.timeScale != 1.0f);
            if (ImGui.Checkbox("Slow motion [tab]", ref slow_motion))
            {
                Time.timeScale = (Time.timeScale == 1.0f) ? 0.1f : 1.0f;
            }
        }
        ImGui.End();
    }
}
