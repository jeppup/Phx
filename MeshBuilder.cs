using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class MeshBuilder : MonoBehaviour {
    private Vector3[] mesh_verticies_original;
    private Mesh mesh;

    int x_dim = 20;
    int z_dim = 20;

    // Use this for initialization
    void Awake () {
        mesh = GetComponent<MeshCollider>().sharedMesh;

        mesh.SetVertices(buildVerticies());
        mesh.SetTriangles(buildIndicies(), 0);
        mesh.SetNormals(buildNormals());
    }

    private List<Vector3> buildNormals()
    {
        Vector3[] normals = new Vector3[x_dim * z_dim];
        for(int i = 0; i < x_dim; i++)
        {
            for( int j = 0; j < z_dim; j++)
            {
                normals[i * z_dim + j] = new Vector3(0.0f, 1.0f, 0.0f);
            }
        }

        return normals.ToList();
    }

    private int[] buildIndicies()
    {
        int tri_count = x_dim * z_dim * 2 * 3;
        int[] triangles = new int[tri_count];

        int running_idx = 0;
        for(int i = 0; i < x_dim - 1; i++)
        {
            for(int j = 0; j < z_dim - 1; j++)
            {
                buildTopLeftTri(triangles, i, j, running_idx);
                buildBottomRightTri(triangles, i, j, running_idx + 3);

                running_idx += 6;
            }
        }

        return triangles;
    }

    private void buildTopLeftTri(int[] indicies, int i, int j, int idx)
    {
        indicies[idx + 0] = i*z_dim + j;
        indicies[idx + 1] = i * z_dim + j + 1;
        indicies[idx + 2] = (i + 1) * z_dim + j;
    }

    private void buildBottomRightTri(int[] indicies, int i, int j, int idx)
    {
        indicies[idx + 0] = i * z_dim + j + 1;
        indicies[idx + 1] = (i + 1) * z_dim + j + 1;
        indicies[idx + 2] = (i + 1) * z_dim + j;
    }

    private List<Vector3> buildVerticies()
    {
        var verticies = new Vector3[x_dim * z_dim];
        float dx = 1.0f / x_dim;
        float dz = 1.0f / z_dim;

        for (int i = 0; i < x_dim; i++)
        {
            for (int j = 0; j < z_dim; j++)
            {
                float x = i * dx;
                float z = j * dz;

                verticies[i * z_dim + j] = new Vector3(x, 0.0f, z);
            }
        }

        return verticies.ToList();
    }
	
	// Update is called once per frame
	void Update () {
		
	}
}
