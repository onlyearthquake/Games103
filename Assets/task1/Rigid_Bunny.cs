using UnityEngine;
using System.Collections;
using System.Collections.Generic;
public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	[SerializeField]
	Vector3 v 			= new Vector3(0, 0, 0); // velocity
	[SerializeField]
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;
	[SerializeField]
	float restitution 	= 0.5f;                 // for collision

	float uT = 0.7f;
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
	Matrix4x4 Add_Matrix(Matrix4x4 a,Matrix4x4 b)
    {
		for (int i = 0;i <4;i++)
        {
			for(int j = 0; j < 4; j++)
            {
				a[i, j] += b[i, j];
            }
        }
		return a;
    }
	Matrix4x4 multiply(Matrix4x4 a,float c)
    {

		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				a[i, j] *= c;
			}
		}
		return a;
	}
	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		//Detect Has Collide
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;
		List<Vector3> collision_points = new List<Vector3>();
		Vector3 center = transform.position;
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		for (int i = 0;i < vertices.Length; i++)
        {
			//Signed Distance
			Vector3 d = (center + (Vector3)(R * vertices[i]) - P);
			float distance = Vector3.Dot(d, N);
			if(distance < 0)
            {
				//negative ,has colision
				collision_points.Add(vertices[i]);
            }
        }
		//No Collision
		if(collision_points.Count <= 0)
			return;

		//calculate average point
		Vector3 p_i = Vector3.zero;
		Vector3 v_i = Vector3.zero;
		int c = 0;
		for (int i = 0; i < collision_points.Count; i++)
		{
			Vector3 temp_v_i = v + (Vector3)(Vector3.Cross(w, R * collision_points[i]));
			if (Vector3.Dot(temp_v_i, N) < 0f)
			{
				c++;
				v_i += temp_v_i;
				p_i += (Vector3)(R * collision_points[i]);
			}
		}
		if (c <= 0)
			return;
		p_i /= c;
		v_i /= c;

		//split
		Vector3 v_i_N = Vector3.Dot(v_i, N) * N;
		Vector3 v_i_T = v_i - v_i_N;
		float a = Mathf.Max(0, 1 - uT * (1 + restitution) * (v_i_N.magnitude / v_i_T.magnitude));
		Vector3 v_i_new = -1 * restitution * v_i_N + v_i_T * a;
		Vector3 delta_v = (v_i_new - v_i);

		Matrix4x4 K = Matrix4x4.identity;
		K = multiply(K, 1.0f / mass);
		K = Add_Matrix(K,multiply(Get_Cross_Matrix( p_i) * (R * I_ref * R.transpose).inverse * Get_Cross_Matrix( p_i),-1));

		//冲量
		Vector3 J = K.inverse * delta_v;
		
		v += (J / mass);
		w += (Vector3)((R * I_ref * R.transpose).inverse * Get_Cross_Matrix(p_i) * J);

	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f; 
			uT = 0.7f;
			launched =false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}
		// Part I: Update velocities
		//Gravity
		Vector3 F = new Vector3(0, -9.8f,0);

		v += F * dt;

		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		v *= linear_decay;
		w *= angular_decay;
		// Part III: Update position & orientation
		Vector3 x    = transform.position;
		x += (v * dt);

		Quaternion q = transform.rotation;
		q *= Quaternion.Euler(w.x * dt, w.y * dt, w.z * dt );
		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q;
		if (v.magnitude < 0.3f)
		{
			restitution *= 0.5f;
		}
		
	}
}
