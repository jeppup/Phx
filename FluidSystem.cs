using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;

public class FluidSystem : MonoBehaviour {
    public GameObject particle_model;
    Particle[] particles;
    public int rows;
    public int cols;
    int particle_count;
    public float particle_mass;
    public float particle_density;
    public float h = 0.5f;    // Kernel smoothing bandwidth
    public float k = 0.5f;    // Ideal gas equation thermal coefficient


	// Use this for initialization
	void Start () {
        particle_count = rows * cols;
        particles = new Particle[particle_count];

        for ( int i = 0; i < rows; i++)
        {
            for( int j = 0; j < cols; j++)
            {
                int idx = rows * i + j;
                Vector3 position = new Vector3(0.5f * i, 0, 0.5f * j);
                Vector3 velocity = new Vector3(0, 0, 0);
                particles[idx] = new Particle(particle_mass, particle_density, position, velocity);
                particles[idx].Model = GameObject.Instantiate(particle_model);
                particles[idx].Model.transform.position = position;
                Rigidbody rigidBody = particles[idx].Model.GetComponent(typeof(Rigidbody)) as Rigidbody;
                rigidBody.position = position;
                rigidBody.velocity = velocity;

            }
        }

        UpdateDensity();
        foreach(var P in particles)
        {
            P.d_0 = P.d;
        }
    }

    // Update is called once per frame
    void FixedUpdate () {
        foreach( var P in particles)
        {
            P.LoadPositionFromModel();
        }
        
        UpdateDensity();
        UpdatePressure();
        UpdateForce();
        solve(Time.deltaTime );
    }

    // A_i = sum{ M_j * (A_j / d_j) * W(X_i, X_j)}

    // V = m / d
    // d_i = sum{ M_j * (d_j / d_j) * W(X_i, X_j)} = sum{ M_j * W(X_i, X_j)}
    void UpdateDensity()
    {
        foreach (var Pi in particles)
        {
            var density = 0.0f;
            foreach (var Pj in particles)
            {
                if (Pi == Pj) continue;
                float r = (Pi.X - Pj.X).magnitude;
                if (r > 2 * h) continue;

                density += Pj.m * W(r);
            }
            Pi.d = density;
        }
    }

    // Modified ideal gas equation
    // p = k * (d - d_0)
    void UpdatePressure()
    {
        foreach (var Pi in particles)
        {
            Pi.p = k * (Pi.d - Pi.d_0);
        }
    }

    void UpdateForce()
    {
        foreach( var Pi in particles)
        {
            Pi.F.Set(0, 0, 0);
            Pi.F += F_pressure(Pi);
            Pi.F += F_viscosity(Pi);
        }
    }

    Vector3 F_pressure( Particle P )
    {
        Vector3 F_p = Vector3.zero;
        foreach( var Pj in particles)
        {
            if (Pj == P) continue;
            float r = (P.X - Pj.X).magnitude;
            if (r > 2 * h) continue;

            F_p -= Pj.m * (( P.p + Pj.p) / 2* Pj.d) * W_gradient(r, P, Pj); 
        }
        return F_p;
    }

    Vector3 F_viscosity( Particle P )
    {
        return Vector3.zero;
    }


    float W( float r )
    {
        float scale = 1 / (Mathf.Pow(Mathf.PI, 3 / 2) * h * h * h);
        return scale * Mathf.Exp(-(r * r) / (h * h)) ;
    }

    Vector3 W_gradient( float r, Particle p0, Particle p1)
    {
        Vector3 result = Vector3.zero;
        float scale = 1 / (Mathf.Pow(Mathf.PI, 3f / 2f) * h * h * h);
        result.x = -scale * Mathf.Exp(-r * r / (h * h)) * 2f * (p0.X.x - p1.X.x) / (h * h);
        result.y = -scale * Mathf.Exp(-r * r / (h * h)) * 2f * (p0.X.y - p1.X.y) / (h * h);
        result.z = -scale * Mathf.Exp(-r * r / (h * h)) * 2f * (p0.X.z - p1.X.z) / (h * h);

        return result;
    }

    // INCORRECT
    float W_laplacian( float r )
    {
        //float r = (p0.X - p1.X).magnitude;
        //float scale = 1 / (Mathf.Pow(Mathf.PI, 3 / 2) * h * h * h);
        //return scale * 2 * Mathf.Exp(-r * r / (h * h)) * (h * h + 2 * r * r) / (h * h * h * h);
        return 0f;
    }

    void solve(float dt)
    {
        Matrix<float> M = Compute_M();
        Matrix<float> dFdY = Compute_dFdY();
        Vector<float> F = Compute_F();

        Vector<float> deltaY = ((M / dt) - dFdY).Solve(F);

        int particle_index = 0;
        foreach( var P in particles)
        {
            Vector3 deltaX = toUnityVector(deltaY.SubVector(particle_index, 3));
            Vector3 deltaV = toUnityVector(deltaY.SubVector(particle_count * 3 + particle_index, 3));
            P.X += deltaX;
            P.V += deltaV;
            P.applyChanges();
            particle_index += 3;
        }

    }

    Vector<float> Compute_F()
    {
        int system_size = particle_count * 6;
        Vector<float> F = Vector<float>.Build.Dense(system_size);
        int particle_index = 0;
        foreach( var P in particles)
        {
            // Fx
            F.SetSubVector(particle_index, 3, toMathLibVector(P.V));
            // Fv
            F.SetSubVector(3 * particle_count + particle_index, 3, toMathLibVector(P.F));
            particle_index += 3;
        }
        return F;
    }

    Matrix<float> Compute_dFdY()
    {
        int system_size = particle_count * 6;
        return Matrix<float>.Build.Dense(system_size, system_size);
    }

    Matrix<float> Compute_M()
    {
        int system_size = particle_count * 6;
        Matrix<float> M = Matrix<float>.Build.Dense(system_size, system_size);
        M.SetSubMatrix(0, particle_count * 3, 0, particle_count * 3, Matrix<float>.Build.DenseIdentity(particle_count * 3, particle_count * 3));
        int particle_index = particle_count * 3;
        foreach (var P in particles)
        {
            M.SetSubMatrix(particle_index, 3, particle_index, 3, P.m * Matrix<float>.Build.DenseIdentity(3, 3));
            particle_index += 3;
        }

        return M;
    }

    Vector<float> toMathLibVector( Vector3 vec)
    {
        Vector<float> c_vec = Vector<float>.Build.Dense(3);
        c_vec[0] = vec.x;
        c_vec[1] = vec.y;
        c_vec[2] = vec.z;
        return c_vec;
    }

    Vector3 toUnityVector( Vector<float> vec)
    {
        Vector3 c_vec;
        c_vec.x = vec[0];
        c_vec.y = vec[1];
        c_vec.z = vec[2];

        return c_vec;
    }
}

public class Particle
{
    public Particle( float mass, float density, Vector3 position, Vector3 velocity)
    {
        m = mass;
        d_0 = density;
        d = density;
        X = position;
        V = velocity;
    }
    public GameObject Model { get; set; }

    public Vector3 X { get; set; }
    public Vector3 V { get; set; }
    public Vector3 F { get; set; }
    public float m { get; set; }
    public float d { get; set; }
    public float d_0 { get; set; }
    public float p { get; set; }

    public void LoadPositionFromModel()
    {
        Rigidbody rigidBody = Model.GetComponent(typeof(Rigidbody)) as Rigidbody;
        X = rigidBody.position;
        V = rigidBody.velocity;
    }

    public void applyChanges()
    {
        Rigidbody rigidBody = Model.GetComponent(typeof(Rigidbody)) as Rigidbody;
        rigidBody.AddForce(F);
        //Model.transform.position.Set(X.x, X.y, X.z);

    }

}

public class Solver_BackwardEuler
{

}
