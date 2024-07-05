using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class Ground : MonoBehaviour
{
    public Verlet.System branches = new Verlet.System();
    public int numSegments = 40; // Number of branch segments
    public float minX = 2.0f; // Minimum horizontal distance between points
    public float maxX = 6.0f; // Maximum horizontal distance between points
    public float minY = -3.0f; // Minimum vertical distance between points
    public float maxY = 3.0f; // Maximum vertical distance between points
    public float minYClamp = -2.5f; // Minimum Y value clamp
    public float maxYClamp = 2.5f; // Maximum Y value clamp

    void Start()
    {
        GenerateBranches();
    }

    void GenerateBranches()
    {
        float x = 0;
        float y = 0;
        for (int i = 0; i < numSegments + 1; ++i)
        {
            branches.AddPoint(new float3(x, y, 0), "branch");
            x += UnityEngine.Random.Range(minX, maxX);
            y += UnityEngine.Random.Range(minY, maxY);
            y = math.clamp(y, minYClamp, maxYClamp); //Makes sure we stay on screen
        }
        for (int i = 0; i < numSegments; ++i)
        {
            branches.AddBone("branch", i, i + 1);
        }
    }

    public float BranchHeight(float x, int start_id, int end_id)
    {
        var start = branches.points[start_id];
        var end = branches.points[end_id];
        float branch_t = (x - start.bind_pos[0]) / (end.bind_pos[0] - start.bind_pos[0]);
        return math.lerp(start.pos[1], end.pos[1], branch_t);
    }

    public float BranchesHeight(float x)
    {
        for (int i = 0; i < branches.bones.Count; ++i)
        {
            var point_ids = branches.bones[i].points;
            if (x >= branches.points[point_ids[0]].pos[0] && x < branches.points[point_ids[1]].pos[0])
            {
                return BranchHeight(x, point_ids[0], point_ids[1]);
            }
        }
        if (x < 0.0f)
        {
            return branches.points[0].pos[1];
        }
        else
        {
            return branches.points[branches.points.Count - 1].pos[1];
        }
    }

    void Update()
    {
        branches.DrawBones(Color.green);
    }
}

