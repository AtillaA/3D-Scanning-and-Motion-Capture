#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"
#include "VirtualSensor.h"

using namespace std;


struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;

	// color stored as 4 unsigned char
	Vector4uc color;
};


bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// ---------------------------------------------------------------------------------------------------------------------------
	// TODO 2: Use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - [hint] for debugging, first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) 
	// - note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0)
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width * height;

	// TODO: Determine number of valid faces
	unsigned nFaces = width * height /2;
	// ---------------------------------------------------------------------------------------------------------------------------

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// ---------------------------------------------------------------------------------------------------------------------------
	// TODO: save vertices
	// Store the vertices with color values
	for (int idx = 0; idx < nVertices; idx++) {
		if (vertices[idx].position.x() == MINF || 
			vertices[idx].position.y() == MINF || 
			vertices[idx].position.z() == MINF) {
			outFile << "0 0 0 0 0 0" << std::endl;
		} else {
			outFile << vertices[idx].position.x() << " " << vertices[idx].position.y() << " " << vertices[idx].position.z() 
					<< " " << vertices[idx].color.x() << " " << vertices[idx].color.y() << " " << vertices[idx].color.z() 
					<< " " << 255 << std::endl;
		}
	}

	// TODO: save valid faces
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {

			int currIdx = i * width + j;

			// if the vertice is not invalid
			if (vertices[currentIdx].position.x() != MINF && 
				vertices[currentIdx].position.y() != MINF && 
				vertices[currentIdx].position.z() != MINF) {
				int northIdx = (i - 1) * width + j;
				int southIdx = (i + 1) * width + j;
				int eastIdx = i * width + j + 1;
				int westIdx = i * width + j - 1;
				
				if (isValidIndex(eastIdx, nVertices) && isValidIndex(southIdx, nVertices)) {
					if (canBeAFace(vertices[currentIdx].position, vertices[eastIdx].position, 
									vertices[bottomIdx].position, edgeThreshold)) {
						outFile << "3 " << currIdx << " " << southIdx << " " << eastIdx << std::endl;
					}
				}

				if (isValidIndex(westIdx, nVertices) && isValidIndex(northIdx, nVertices)) {
					if (canBeAFace(vertices[currentIdx].position, vertices[westIdx].position, 
									vertices[northIdx].position, edgeThreshold)) {
						outFile << "3 " << currIdx << " " << northIdx << " " << westIdx << std::endl;
					}
				}
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------

	// close file
	outFile.close();

	return true;
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../data/rgbd_dataset_freiburg1_xyz/";
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
		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];

		// --------------------------------------------------------------------------------------------------------------------------------------
		// TODO #1: Back-projection
		// Write result to the vertices array below (keep ordering), if the depth is invalid (MINF) write the following to vertices array
		// Otherwise apply back-projection and transform the vertex to world space, use the corresponding color from colormap
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);

		// Loop through every pixel (i -> y pixels, j -> x pixels)
		for (int i = 0; i < sensor.GetDepthImageHeight(); i++) {
			for (int j = 0; j < sensor.GetDepthImageWidth(); j++) {
				// Get the index
				int idx = i * sensor.GetDepthImageWidth() + j;

				if (depthMap[idx] == MINF) { // if depth is invalid, assign predetermined vlaues
					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[idx].color = Vector4uc(0, 0, 0, 0);
				} else {
					float x = ((float)(j) - cX) / fovX;
					float y = ((float)(i) - cY) / fovY;

					// apply back-projection and transform the vertex to world space,
					Vector4f cameraSytemCoordinates = Vector4f(depthMap[idx]*x, depthMap[idx]*y, depthMap[idx], 1); // camera coordinate system
					Vector4f worldSpaceCoordinates = trajectoryInv * cameraSytemCoordinates; // camera transformation

					// Assign to vertex
					vertices[idx].position = worldSpaceCoordinates;
					vertices[idx].color = Vector4uc(colorMap[idx*4], colorMap[idx*4 + 1], colorMap[idx*4 + 2], colorMap[idx*4 + 3]);
				}
			}
		}
		// --------------------------------------------------------------------------------------------------------------------------------------

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
