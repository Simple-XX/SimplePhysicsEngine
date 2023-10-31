
/**
* @file Grid.h
* @briefGrid  头文件
* @author Zone.N (Zone.Niuzh@hotmail.com)
* @version 1.0
* @date 2023-10-31
* @copyright MIT LICENSE
* https://github.com/Simple-XX/SimplePhysicsEngine
* @par change log:
* <table>
* <tr><th>Date<th>Author<th>Description
* <tr><td>2023-10-31<td>Zone.N<td>迁移到 doxygen
* </table>
*/

#pragma once

#include <vector>
#include "Particle.h"

typedef std::vector<int> Cell;

class Grid
{
public:
	Grid();

	void updateStructure(std::vector<Particle> &particles);

	std::vector<Cell> getNeighboringCells(Eigen::Vector2f position);
	
private:
	int numberCellsX;
	int numberCellsY;

	std::vector<std::vector<Cell>> cells;
};

