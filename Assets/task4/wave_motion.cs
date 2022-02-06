using UnityEngine;
using System.Collections;

public class wave_motion : MonoBehaviour 
{
	int size 		= 100;
	float rate 		= 0.005f;
	float gamma		= 0.004f;
	float damping 	= 0.996f;
	float[,] 	old_h;
	float[,]	low_h;
	float[,]	vh;
	float[,]	b;

	bool [,]	cg_mask;
	float[,]	cg_p;
	float[,]	cg_r;
	float[,]	cg_Ap;
	bool 	tag=true;

	Vector3 	cube_v = Vector3.zero;
	Vector3 	cube_w = Vector3.zero;

	Transform block;
	Bounds block_bounds;
	Transform cube;
	Bounds cube_bounds;
	// Use this for initialization
	void Start () 
	{
		GameObject block_obj = GameObject.Find("Block");
		block = block_obj.transform;
		block_bounds = block_obj.GetComponent<BoxCollider>().bounds;
		GameObject cube_obj = GameObject.Find("Cube");
		cube = cube_obj.transform;
		cube_bounds = cube_obj.GetComponent<BoxCollider>().bounds;
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.Clear ();

		Vector3[] X=new Vector3[size*size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			X[i*size+j].x=i*0.1f-size*0.05f;
			X[i*size+j].y=0;
			X[i*size+j].z=j*0.1f-size*0.05f;
		}

		int[] T = new int[(size - 1) * (size - 1) * 6];
		int index = 0;
		for (int i=0; i<size-1; i++)
		for (int j=0; j<size-1; j++)
		{
			T[index*6+0]=(i+0)*size+(j+0);
			T[index*6+1]=(i+0)*size+(j+1);
			T[index*6+2]=(i+1)*size+(j+1);
			T[index*6+3]=(i+0)*size+(j+0);
			T[index*6+4]=(i+1)*size+(j+1);
			T[index*6+5]=(i+1)*size+(j+0);
			index++;
		}
		mesh.vertices  = X;
		mesh.triangles = T;
		mesh.RecalculateNormals ();

		low_h 	= new float[size,size];
		old_h 	= new float[size,size];
		vh 	  	= new float[size,size];
		b 	  	= new float[size,size];

		cg_mask	= new bool [size,size];
		cg_p 	= new float[size,size];
		cg_r 	= new float[size,size];
		cg_Ap 	= new float[size,size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			low_h[i,j]=99999;
			old_h[i,j]=0;
			vh[i,j]=0;
		}
	}

	void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
	{
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			Ax[i,j]=0;
			if(i!=0)		Ax[i,j]-=x[i-1,j]-x[i,j];
			if(i!=size-1)	Ax[i,j]-=x[i+1,j]-x[i,j];
			if(j!=0)		Ax[i,j]-=x[i,j-1]-x[i,j];
			if(j!=size-1)	Ax[i,j]-=x[i,j+1]-x[i,j];
		}
	}

	float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
	{
		float ret=0;
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			ret+=x[i,j]*y[i,j];
		}
		return ret;
	}

	void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
	{
		//Solve the Laplacian problem by CG.
		A_Times(mask, x, cg_r, li, ui, lj, uj);

		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			cg_p[i,j]=cg_r[i,j]=b[i,j]-cg_r[i,j];
		}

		float rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);

		for(int k=0; k<128; k++)
		{
			if(rk_norm<1e-10f)	break;
			A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
			float alpha=rk_norm/Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				x[i,j]   +=alpha*cg_p[i,j];
				cg_r[i,j]-=alpha*cg_Ap[i,j];
			}

			float _rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);
			float beta=_rk_norm/rk_norm;
			rk_norm=_rk_norm;

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				cg_p[i,j]=cg_r[i,j]+beta*cg_p[i,j];
			}
		}

	}

	void Shallow_Wave(float[,] old_h, float[,] h, float[,] new_h)
	{
		//Step 1:
		//TODO: Compute new_h based on the shallow wave model.
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{

				new_h[i, j] = h[i, j];
				new_h[i, j] += (h[i, j] - old_h[i, j]) * damping;
				if (i > 0)
					new_h[i, j] += rate * (h[i - 1, j] - h[i, j]);
				if (i < size - 1)
					new_h[i, j] += rate * (h[i + 1, j] - h[i, j]);
				if (j > 0)
					new_h[i, j] += rate * (h[i, j - 1] - h[i, j]);
				if (j < size - 1)
					new_h[i, j] += rate * (h[i, j + 1] - h[i, j]);
			}
		}
		//Step 2: Block->Water coupling
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				cg_mask[i, j] = true;
				b[i, j] = 0f;
				vh[i, j] = 0f;
			}
		}
		//TODO: for block 1, calculate low_h.
		//TODO: then set up b and cg_mask for conjugate gradient.
		Vector3 block_pos = cube.position;
		cube_bounds = cube.GetComponent<BoxCollider>().bounds;
		int li = (int)((block_pos.x + 4.5f) / 0.1f), ui = (int)((block_pos.x + 5.5f) / 0.1f);
		int lj = (int)((block_pos.z + 4.5f) / 0.1f), uj = (int)((block_pos.z + 5.5f) / 0.1f);
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] X = mesh.vertices;
		li = Mathf.Clamp( li, 0, size - 1);
		lj = Mathf.Clamp(lj, 0, size - 1);
		ui = Mathf.Clamp(ui, 0, size - 1);
		uj = Mathf.Clamp(uj, 0, size - 1);
		for (int i = li;i <= ui; i++)
        {
			for(int j = lj;j <= uj; j++)
            {
				float distance = 100;
				Vector3 pos = X[i * 100 + j];
				Vector3 center = new Vector3(pos.x, -10f, pos.z);
				Ray detect_ray = new Ray(center, new Vector3(0,1,0));
				cube_bounds.IntersectRay(detect_ray,out distance);
				low_h[i, j] = distance - 10;
				
				if (low_h[i, j] < h[i, j])
				{
					b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
					cg_mask[i, j] = true;
				}
				
            }
		}
		//TODO: Solve the Poisson equation to obtain vh (virtual height).
		Conjugate_Gradient(cg_mask, b, vh, li, ui, lj, uj);
		//TODO: for block 2, calculate low_h.
		//TODO: then set up b and cg_mask for conjugate gradient.
		block_pos = block.position;
		block_bounds = block.GetComponent<BoxCollider>().bounds;
		li = Mathf.Clamp((int)((block_pos.x + 4.5f) / 0.1f),0,size - 1);
		ui = Mathf.Clamp( (int)((block_pos.x + 5.5f) / 0.1f), 0, size - 1);
		lj = Mathf.Clamp((int)((block_pos.z + 4.5f) / 0.1f), 0, size - 1);
		uj = Mathf.Clamp((int)((block_pos.z + 5.5f) / 0.1f), 0, size - 1);
		for (int i = li; i <= ui; i++)
		{
			for (int j = lj; j <= uj; j++)
			{
				float distance = 100;
				Vector3 pos = X[i * 100 + j];
				Vector3 center = new Vector3(pos.x, -10f, pos.z);
				Ray detect_ray = new Ray(center, new Vector3(0,1,0));
				block_bounds.IntersectRay(detect_ray, out distance);
				low_h[i, j] = distance - 10;
				
				if (low_h[i, j] < h[i, j])
				{
					b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
					cg_mask[i, j] = true;
				}
				
			}
		}
		//TODO: Solve the Poisson equation to obtain vh (virtual height).
		Conjugate_Gradient(cg_mask, b, vh, li, ui, lj, uj);
		//TODO: Diminish vh.
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				if (cg_mask[i, j])
				{
					vh[i, j] *= gamma;
				}
			}
		}
		//TODO: Update new_h by vh.

		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				if (i > 0)
					new_h[i, j] += rate * (vh[i - 1, j] - vh[i, j]);
				if (i < size - 1)
					new_h[i, j] += rate * (vh[i + 1, j] - vh[i, j]);
				if (j > 0)
					new_h[i, j] += rate * (vh[i, j - 1] - vh[i, j]);
				if (j < size - 1)
					new_h[i, j] += rate * (vh[i, j + 1] - vh[i, j]);
			}
		}
		//Step 3
		//TODO: old_h <- h; h <- new_h;

		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				old_h[i, j] = h[i, j];
				h[i, j] = new_h[i, j];
			}
		}
		//Step 4: Water->Block coupling.
		//More TODO here.
	}
	

	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X    = mesh.vertices;
		float[,] new_h = new float[size, size];
		float[,] h     = new float[size, size];

		//TODO: Load X.y into h.
		for(int i = 0;i < size; i++)
        {
			for(int j = 0;j < size;j++)
            {
				h[i, j] = X[i * size + j].y;
            }
        }
		if (Input.GetKeyDown ("r")) 
		{
			//TODO: Add random water.
			int pos_x = Random.Range(0, size), pos_y = Random.Range(0, size);
			float r = Random.Range(0f, 1.0f);
			h[pos_x, pos_y] += r;
			int[,] dir = new int[4,2] { { 0, 1 },{ 0, -1 },{ 1, 0 },{ -1, 0 } };
			while (true)
			{
				int random_dir = Random.Range(0, 4);
				int x_new = pos_x + dir[random_dir, 0], y_new = pos_y + dir[random_dir, 1];
				if (x_new < 0 || x_new >= size || y_new < 0 || y_new >= size)
					continue;
				h[x_new, y_new] -= r;
				break;
			}
		}
	
		for(int l=0; l<8; l++)
		{
			Shallow_Wave(old_h, h, new_h);
		}

		//TODO: Store h back into X.y and recalculate normal.
		for(int i = 0;i < size; i++)
        {
			for(int j = 0;j < size; j++)
            {
				X[i * size + j].y = h[i, j];
            }
        }
		mesh.vertices = X;
		mesh.RecalculateNormals();
		
	}
}
