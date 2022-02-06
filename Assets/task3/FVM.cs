using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
	float dt 			= 0.003f;
    float mass 			= 1;
	float stiffness_0	= 20000.0f;
    float stiffness_1 	= 5000.0f;
    float damp			= 0.999f;

	int[] 		Tet;
	int tet_number;			//The number of tetrahedra

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				//The number of vertices

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num; 
    float blendAlpha = 0.5f;
    float uT = 0.5f;
    float uN = 0.5f;
    SVD svd = new SVD();

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/task3/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/task3/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center=center/number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y;
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}
		}
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


        //Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();


		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

        //TODO: Need to allocate and assign inv_Dm
        inv_Dm = new Matrix4x4[tet_number];
        for(int i = 0;i < tet_number; i++)
        {
            inv_Dm[i] = Build_Edge_Matrix(i).inverse;
        }
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
        //TODO: Need to build edge matrix here.
        Vector3 X_0 = X[Tet[tet * 4]], X_1 = X[Tet[tet * 4 + 1]], X_2 = X[Tet[tet * 4 + 2]], X_3 = X[Tet[tet * 4 + 3]];
        ret.SetColumn(0, X_1 - X_0);
        ret.SetColumn(1, X_2 - X_0);
        ret.SetColumn(2, X_3 - X_0);
        //3x3转4x4矩阵记得加上1 不然求逆等运算不等效
        ret[3,3] = 1;
        return ret;
    }


    void _Update()
    {
        //smooth
        for (int i = 0; i < number; i++)
        {
            V_sum[i] = Vector3.zero;
            V_num[i] = 0;
        }
        for (int tet = 0; tet < tet_number; tet++)
        {
            int p_0 = Tet[tet * 4 + 0], p_1 = Tet[tet * 4 + 1], p_2 = Tet[tet * 4 + 2], p_3 = Tet[tet * 4 + 3];
            V_sum[p_0] += (V[p_1] + V[p_2] + V[p_3]);
            V_num[p_0] += 3;
            V_sum[p_1] += (V[p_2] + V[p_3] + V[p_0]);
            V_num[p_1] += 3;
            V_sum[p_2] += (V[p_1] + V[p_3] + V[p_0]);
            V_num[p_2] += 3;
            V_sum[p_3] += (V[p_1] + V[p_2] + V[p_0]);
            V_num[p_3] += 3;
        }
        for (int i = 0; i < number; i++)
        {
            V[i] = blendAlpha * V[i] + (1 - blendAlpha) * V_sum[i] / V_num[i];
        }
        // Jump up.
        if (Input.GetKeyDown(KeyCode.Space))
        {
            for (int i = 0; i < number; i++)
                V[i].y += 0.2f;
        }

        for (int i = 0; i < number; i++)
        {
            //TODO: Add gravity to Force.
            Force[i] =new Vector3(0,-9.8f,0);
        }
        for (int tet=0; tet<tet_number; tet++)
    	{
            int p_0 = Tet[tet * 4 + 0], p_1 = Tet[tet * 4 + 1], p_2 = Tet[tet * 4 + 2], p_3 = Tet[tet * 4 + 3];
            //TODO: Deformation Gradient
            Matrix4x4 F = Build_Edge_Matrix(tet);
            F = F * inv_Dm[tet];
            F[3, 3] = 1;
            //TODO: Green Strain
            Matrix4x4 G = Multiply_Matrix(Add_Matrix(F.transpose * F,
                          Multiply_Matrix(Matrix4x4.identity,-1f)),0.5f);
            G[3, 3] = 1;
            //TODO: Second PK Stress
            Matrix4x4 S = Add_Matrix(Multiply_Matrix(G, stiffness_1 * 2),
                            Multiply_Matrix(Matrix4x4.identity, Trace(G) * stiffness_0));
            S[3, 3] = 1;
            //TODO: Elastic Force
            Matrix4x4 P = F * S;
            P[3, 3] = 1;
            Matrix4x4 M_Force = Multiply_Matrix( P * inv_Dm[tet].transpose, -1 / (inv_Dm[tet].determinant * 6));
            Force[p_1] += (Vector3)(M_Force.GetColumn(0));
            Force[p_2] += (Vector3)(M_Force.GetColumn(1));
            Force[p_3] += (Vector3)(M_Force.GetColumn(2));
            Force[p_0] += -1f * (Vector3)(M_Force.GetColumn(0) + M_Force.GetColumn(1) + M_Force.GetColumn(2));
        }
        
        
        for (int i = 0; i < number; i++)
        {
            //TODO: Update X and V here.
            V[i] += (Force[i] / mass) * dt;
            V[i] *= damp;
            X[i] += V[i] * dt;
            //TODO: (Particle) collision with floor.
            if (X[i].y < -3)
            {
                //like lab1
                X[i].y = -3;
                Vector3 floorNormal = new Vector3(0, 1, 0);
                if(V[i].y < 0)
                {
                    Vector3 V_N = Vector3.Dot(floorNormal, V[i]) * floorNormal;
                    Vector3 V_T = V[i] - V_N;
                    float a = Math.Max(1 - uT * (1 + uN) * V_N.magnitude / V_T.magnitude, 0);
                    V[i] = -uN * V_N  + V_T * a;
                }
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
    	for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }

    Matrix4x4 Add_Matrix(Matrix4x4 a, Matrix4x4 b)
    {
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                a[i, j] += b[i, j];
            }
        }
        return a;
    }
    Matrix4x4 Multiply_Matrix(Matrix4x4 a, float c)
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
    float Trace(Matrix4x4 a)
    {
        float res = 0f;
        for(int i = 0;i < 3; i++)
        {
            res += a.GetColumn(i)[i];
        }
        return res;
    }
}
