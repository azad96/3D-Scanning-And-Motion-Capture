#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"
#include "VirtualSensor.h"

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

bool validateTriangle(Vertex* vertices, int idx1, int idx2, int idx3, float thr)
{
	return (vertices[idx1].position - vertices[idx3].position).norm() < thr
			&& (vertices[idx1].position - vertices[idx2].position).norm() < thr
			&& (vertices[idx2].position - vertices[idx3].position).norm() < thr;
}


bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) 
	// - (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width * height;

	// TODO: Determine number of valid faces
	unsigned nFaces = 0;

	std::vector<Vector3i> triangles;

	for (int x = 0; x < height-1; x++){
		for (int y = 0; y < width-1; y++){
			int idx1 = (x * width) + y; // 0
			int idx2 = (x+1) * width + y; // 5
			int idx3 = (x * width) + y + 1; // 1
			int idx4 = (x+1) * width + y + 1; // 6

			if (validateTriangle(vertices, idx1, idx2, idx3, edgeThreshold)) {
				triangles.push_back(Vector3i(idx1, idx2, idx3));
			}

			if (validateTriangle(vertices, idx2, idx3, idx4, edgeThreshold)) {
				triangles.push_back(Vector3i(idx2, idx4, idx3));
			}
		}
	}
	nFaces = triangles.size();

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;

	outFile << "# numVertices numFaces numEdges" << std::endl;

	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices

	for (int i = 0; i < width*height; i++){
		float x = vertices[i].position(0); 
		float y = vertices[i].position(1); 
		float z = vertices[i].position(2); 
		unsigned int r = vertices[i].color(0);
		unsigned int g = vertices[i].color(1);
		unsigned int b = vertices[i].color(2);
		unsigned int a = vertices[i].color(3);
		
		if ( x == MINF || y == MINF || z == MINF) {
			outFile << "0.0 0.0 0.0 " << r << " " << g << " " << b << " " << a << "\n";
		}
		else {
			outFile << x << " " << y << " " << z << " " << r << " " << g << " " << b << " " << a << "\n";;
		}
	}

	// TODO: save valid faces
	std::cout << "# list of faces" << std::endl;
	std::cout << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;

	for (int i = 0; i < nFaces; i++){
		Vector3i face = triangles[i];
		outFile << "3 " << face(0) << " " << face(1) << " " << face(2) << "\n";
	}

	// close file
	outFile.close();

	return true;
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../../Data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();
		
		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];

		int width = sensor.GetDepthImageWidth();
		int height = sensor.GetDepthImageHeight();

		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height; y++) {
				int idx = y * width + x;

				// X = x'; Y = y'; Z = z'
				float Z = depthMap[idx];

				if (Z == MINF) {
					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[idx].color = Vector4uc(0, 0, 0, 0);
				}
				else {
					float X = x * Z;
					float Y = y * Z;
					Vector3f imagePlanePoint(X, Y, Z);
					Vector3f depthCameraSpacePoint = depthIntrinsicsInv * imagePlanePoint;
					Vector4f homoDepthCameraSpacePoint(
						depthCameraSpacePoint(0), 
						depthCameraSpacePoint(1),
						depthCameraSpacePoint(2),
						1.0
					);
					Vector4f sensorCameraSpacePoint = depthExtrinsicsInv * homoDepthCameraSpacePoint;
					Vector4f worldSpacePoint = trajectoryInv * sensorCameraSpacePoint;

					vertices[idx].position = worldSpacePoint;

					// r, g, b, a
					vertices[idx].color = Vector4uc(
						colorMap[4 * idx],
						colorMap[4 * idx + 1],
						colorMap[4 * idx + 2],
						colorMap[4 * idx + 3]
					);
				}
			}
		}
		
		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}