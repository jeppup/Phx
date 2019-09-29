using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections;
using UnityEngine;


namespace Assets
{
    public class PdeProblem
    {
        float[,] u;       // Solution array
        float[,] u_t1;    // Solution at t-dt
        float[,] u_t2;    // Solution at t-2dt 

        float[] Ix;       // Boundary conditions in x
        float[] Iz;       // Boundary conditions in z
        float[,] Idt;      // Boundary condition U_tt(x,y,0) = 

        // Discretization dimensions
        int dim_x;
        int dim_z;

        // Boundaries for x [0, a] and for z [0, b]
        float a0;
        float a1;
        float b1;
        float b0;

        float dx;
        float dz;
        float dt;

        const float c = 1.0f;

        // Solves the wave equation(in 2D) for height function U(x, y, t) using the model
        // U_tt = c^2 ( U_xx + U_yy) by discretizing the domain into a grid
        // c = T/s is a constant, where "T" is the tension in the substance and "s" is density

        // Spacial Boundary conditions for x = a, b
        // U(a, z, t) = b_1
        // U(b, z, t) = b_2

        // Spacial boundar conditions for y = c,d 
        // U(x, c, t) = b_3
        // U(x, d, t) = b_4

        // Scale the grid down so that boundaries are [0, 1]
        public PdeProblem( int dim_x, int dim_z, float boundary_a0, float boundary_a1, float boundary_b0, float boundary_b1, float delta_time)
        {
            this.dim_x = dim_x;
            this.dim_z = dim_z;
            //this.a0 = boundary_a0;
            //this.a1 = boundary_a1;
            //this.b0 = boundary_b0;
            //this.b1 = boundary_b1;
            this.a0 = 0.0f;
            this.a1 = 1.0f;
            this.b0 = 0.0f;
            this.b1 = 1.0f;
            this.dx = (a1 - a0) / dim_x;
            this.dz = (b1 - b0) / dim_z;
            this.dt = delta_time;
            InitializeSolutionVectors();
            InitializeBoundaryConditions();
            InitializeInitialConditions();

        }

        public float[,] Step(float dt)
        {
            float r = dt / dx;

            // Interior points
            for( int i = 1; i < dim_x - 1; i++)
            {
                for( int j = 1; j < dim_z - 1; j++)
                {
                    u[i, j] = 2.0f * u_t1[i, j]
                            - u_t2[i, j]
                            + c * r * r * (u_t1[i + 1, j] + u_t1[i - 1, j] + u_t1[i, j + 1] + u_t1[i, j - 1] - 4.0f * u_t1[i, j]);
                            
                }
            }

            // Boundary points
            for( int i = 0; i < dim_x - 1; i++)
            {
                int j = 1;
                u[i, j] = 2.0f * u_t1[i, j] 
                    - u_t2[i, j]
                    + c * r * r * (u_t1[i + 1, j] + u_t1[i + 1, j] + u_t1[i, j + 1] + u_t1[i, j - 1] - 4.0f * u_t1[i, j]);

                j = dim_z - 2;
                u[i, j] = 2.0f * u_t1[i, j]
                    - u_t2[i, j]
                    + c * r * r * (u_t1[i + 1, j] + u_t1[i + 1, j] + u_t1[i, j + 1] + u_t1[i, j - 1] - 4.0f * u_t1[i, j]);
            }

            for (int j = 0; j < dim_z - 1; j++)
            {
                int i = 1;
                u[i, j] = 2.0f * u_t1[i, j]
                    - u_t2[i, j]
                    + c * r * r * (u_t1[i + 1, j] + u_t1[i - 1, j] + u_t1[i, j + 1] + u_t1[i, j + 1] - 4.0f * u_t1[i, j]);

                i = dim_x - 2;
                u[i, j] = 2.0f * u_t1[i, j]
                    - u_t2[i, j]
                    + c * r * r * (u_t1[i + 1, j] + u_t1[i - 1, j] + u_t1[i, j + 1] + u_t1[i, j + 1] - 4.0f * u_t1[i, j]);
            }

            // Move states forward and let u dangle
            u_t2 = u_t1;
            u_t1 = u;

            return u;
        }

        private void InitializeSolutionVectors()
        {
            u = new float[dim_x, dim_z];
            u_t1 = new float[dim_x, dim_z];
            u_t2 = new float[dim_x, dim_z];

            for ( int i = 0; i < dim_x; i++)
            {
                for( int j = 0; j < dim_z; j++)
                {
                    u[i, j] = 0.0f;
                    u_t1[i, j] = 0.0f;
                    u_t2[i, j] = 0.0f;
                }
            }
        }

        private void InitializeInitialConditions()
        {
            // Using the function U(x,z, 0) = xz(2−x)(3−z)
            for ( int i = 0; i < dim_x; i++)
            {
                for( int j = 0; j < dim_x; j++)
                {
                    float x = a0 + i * dx;
                    float z = b0 + j * dz;
                    //u_t2[i, j] = x * z * (2 - x) * (3 - z);
                    //u_t2[i, j] = Mathf.Cos(x - 0.5f) + Mathf.Sin(z - 0.5f);
                    // e−100((x−0.5)2+(y−0.5)2) 
                    u_t2[i, j] = Mathf.Exp( -100 * (Mathf.Pow(x - 0.5f, 2.0f) + Mathf.Pow(z - 0.5f, 2.0f) ));
                }
            }

            // Now using U_t2 and our boundary condition on U_t(x,y,0) to do an F.O. approximation on U_t1
            for (int i = 0; i < dim_x; i++)
            {
                for (int j = 0; j < dim_x; j++)
                {
                    u_t1[i, j] = u_t2[i, j] + dt * Idt[i, j];
                }
            }
        }

        private void InitializeBoundaryConditions()
        {
            Ix = new float[dim_z];
            Iz = new float[dim_x];
            Idt = new float[dim_x, dim_z];

            for( int i = 0; i < dim_z; i++)
            {
                Ix[i] = 0.0f;
            }

            for( int i = 0; i < dim_x; i++)
            {
                Iz[i] = 0.0f;
            }

            for( int i = 0; i < dim_x; i++)
            {
                for( int j = 0; j < dim_z; j++)
                {
                    float x = a0 + i * dx;
                    float z = b0 + j * dz;
                    //Idt[i, j] = 1.5f; // Jävligt magisk siffra?
                    Idt[i, j] = Mathf.Exp(-100 * (Mathf.Pow(x - 0.5f, 2.0f) + Mathf.Pow(z - 0.5f, 2.0f)));
                    //Idt[i, j] = UnityEngine.Random.Range(100.0f, 1.0f); // Jävligt magisk siffra?
                }
            }
        }

        

    }
}
