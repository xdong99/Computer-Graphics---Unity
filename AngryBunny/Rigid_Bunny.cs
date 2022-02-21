using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision

	float mu_T = 0.5f;
	Vector3 G = new Vector3(0.0f, -9.8f, 0.0f);

	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	//define quaternion addition
	Quaternion addTwoQuarternion(Quaternion q0, Quaternion q1) 
    	{
		Quaternion result = new Quaternion(q0.x + q1.x, q0.y + q1.y, q0.z + q1.z, q0.w + q1.w);
		return result;
    	}


	//define matrix multiplication with scalar
	Matrix4x4 multiplyScalar(Matrix4x4 a, float b)  
	{
		Matrix4x4 s = Matrix4x4.zero;
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				s[i, j] = a[i, j] * b;
			}
		}
		return s;
	}

	//define matrix addition
	Matrix4x4 addTwoMatrix(Matrix4x4 a, Matrix4x4 b) 
    	{
		Matrix4x4 s = Matrix4x4.zero;
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				s[i, j] = a[i, j] + b[i, j];
			}
		}
		return s;
	}

	//define matrix subtraction
	Matrix4x4 minusTwoMatrix(Matrix4x4 a, Matrix4x4 b)  
    	{
		//a - b = a + (-1)b
		return addTwoMatrix(a, multiplyScalar(b, -1)); 
    	}

	//In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;
		int[] vid_collision = new int[vertices.Length];
		int num_collision = 0;

		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		
		//For every vertex x_i <-- x + Rr_i, test if φ(x_i) < 0 
		for (int i = 0; i < vertices.Length; i++)
        		{
			Vector3 xi = transform.position + R.MultiplyVector(vertices[i]);
			if(Vector3.Dot(xi - P, N) < 0)
            		{
				vid_collision[num_collision] = i;
				num_collision++;
            		}
        		}
		if (num_collision == 0)
        		{
			return;
        		}

		//v_i <-- v + ω x Rr_i, check if v_i · N < 0
		Vector3 ri = new Vector3(0, 0, 0);

		for (int i = 0; i < num_collision; i++)
		{
			ri += vertices[vid_collision[i]];
		}

		ri = ri / (float)(num_collision);

		Vector3 Rri = R.MultiplyVector(ri);

		Vector3 Vi = v + Vector3.Cross(w, Rri);

		if (Vector3.Dot(Vi, N) >= 0) return;

		//Compute the wanted v_i^new 

		Vector3 VN = Vector3.Dot(Vi, N) * N;

		Vector3 VT = Vi - VN;

		float a = Mathf.Max(0, 1.0f - mu_T * (1.0f + restitution) * Vector3.Magnitude(VN) / Vector3.Magnitude(VT));

		Vector3 ViNew = -1.0f * restitution * VN + a * VT;

		// Compute the impulse j
		Matrix4x4 I_rotation = R * I_ref * Matrix4x4.Transpose(R);
		Matrix4x4 I_inverse = Matrix4x4.Inverse(I_rotation);

		Matrix4x4 Rri_star = Get_Cross_Matrix(Rri);
		Matrix4x4 K = minusTwoMatrix(multiplyScalar(Matrix4x4.identity, (1.0f / mass)), Rri_star * I_inverse * Rri_star);
		Vector3 J = K.inverse.MultiplyVector(ViNew - Vi);

		// update v and ω
		v = v + 1 / mass * J;
		w = w + I_inverse.MultiplyVector(Vector3.Cross(Rri, J));
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}

		// Part I: Update velocities
		if (launched == false) return;

		v = v + dt * G;
		v = v * linear_decay;

		w *= angular_decay;


		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		// Part III: Update position & orientation
		//Update linear status
		Vector3 x    = transform.position;
		x += dt * v;
		//Update angular status
		Quaternion q = transform.rotation;
		Vector3 dw = 0.5f * dt * w;
		Quaternion qua_rotate = new Quaternion(dw.x, dw.y, dw.z, 0.0f);
		q = addTwoQuarternion(q, qua_rotate * q);
		    
		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q;
	}
}