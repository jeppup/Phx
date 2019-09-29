using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Assets;

public class SoftBody : MonoBehaviour {
    private Vector3[] mesh_verticies_original;
    private Mesh mesh;
    private const float material_stiffnes = 0.9f;
    private List<Collision> collisions = new List<Collision>();
    private PdeProblem system;

    int dim_x;
    int dim_z;

    // Use this for initialization
    void Start () {
        mesh = GetComponent<MeshCollider>().sharedMesh;
        mesh_verticies_original = GetComponent<MeshCollider>().sharedMesh.vertices;
        dim_x = (int)Mathf.Sqrt(mesh.vertices.Length);
        dim_z = (int)Mathf.Sqrt(mesh.vertices.Length);

        float boundary_a0 = float.MaxValue;
        float boundary_a1 = float.MinValue;

        float boundary_b0 = float.MaxValue;
        float boundary_b1 = float.MinValue;

        foreach (var v in mesh.vertices)
        {
            if (v.x < boundary_a0)
                boundary_a0 = v.x;
            if (v.x > boundary_a1)
                boundary_a1 = v.x;

            if (v.z < boundary_b0)
                boundary_b0 = v.z;
            if (v.z > boundary_b1)
                boundary_b1 = v.z;
        }

        
        system = new PdeProblem(dim_x, dim_z, boundary_a0, boundary_a1, boundary_b0, boundary_b1, Time.fixedDeltaTime);
    }
	
	// Update is called once per frame
	void Update () {
        Vector3[] updated_vertices = new Vector3[mesh.vertices.Length];
        var new_state = system.Step(Time.fixedDeltaTime);

        for( int i = 0; i < dim_x; i++)
        {
            for( int j = 0; j < dim_z; j++)
            {
                int idx = dim_z * i + j;
                updated_vertices[idx].x = mesh.vertices[idx].x;
                updated_vertices[idx].z = mesh.vertices[idx].z;
                updated_vertices[idx].y = new_state[i, j];
            }
        }

        mesh.vertices = updated_vertices;
	}

    //Vector3[] HandleCollisions(Vector3[] vertices)
    //{
    //    foreach( var c in collisions)
    //    {
    //        foreach (var contact in c.contacts)
    //        {
    //            return system.Step(Time.deltaTime, contact.point);
    //            //ApplyConctact(contact, vertices, c.relativeVelocity);
    //        }
    //    }
    //    collisions.Clear();
    //    return vertices;
    //}

    //void ApplyConctact(ContactPoint cp, Vector3[] vertices, Vector3 velocity)
    //{
    //    var point_of_impact = cp.otherCollider.ClosestPoint(cp.point);
    //    var impacts = ClosestVertexIndex(vertices, point_of_impact);

    //    foreach( var impact in impacts)
    //    {
    //        var debug_force = impact.impact_force * velocity.y * Time.deltaTime;
    //        vertices[impact.mesh_idx].x += impact.impact_force * velocity.x * Time.deltaTime;
    //        vertices[impact.mesh_idx].y += impact.impact_force * velocity.y * Time.deltaTime;
    //        vertices[impact.mesh_idx].z += impact.impact_force * velocity.z * Time.deltaTime;
    //    }
        
    //}

    //struct MeshImpact
    //{
    //    public int mesh_idx;
    //    public float impact_force;
    //}

    //List<MeshImpact> ClosestVertexIndex(Vector3[] vertices, Vector3 point_of_impact)
    //{
    //    List<MeshImpact> impacts = new List<MeshImpact>();

    //    for( int i=0; i < vertices.Length; i++)
    //    {
    //        var distance_i = Mathf.Exp((point_of_impact - vertices[i]).magnitude);
    //        if (  distance_i > 0.1)
    //        {
    //            impacts.Add(new MeshImpact() { mesh_idx = i, impact_force = Mathf.Log(distance_i) });
    //        }
    //    }

    //    return impacts;
    //}

    //private void OnCollisionEnter(Collision collision)
    //{
    //    collisions.Add(collision);
    //}

    //Reset state
    private void OnApplicationQuit()
    {
        GetComponent<MeshCollider>().sharedMesh.vertices = mesh_verticies_original;
    }
}


