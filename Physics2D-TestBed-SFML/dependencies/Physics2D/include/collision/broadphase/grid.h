#ifndef PHYSICS_BROADPHASE_GRID_H
#define PHYSICS_BROADPHASE_GRID_H
#include "../../collision/broadphase/aabb.h"
#include "list"
#include "vector"
namespace Physics2D
{
	class UniformGrid
	{
	public:
		UniformGrid(const real& width, const real& height, const int rows, const int columns);
		std::vector<std::pair<Body*, Body*>> generate();
		std::vector<Body*> raycast(const Vector2& p, const Vector2& d);
		void update(Body* body);
		void insert(Body* body);
		void remove(Body* body);

		int rows()const;
		void setRows(const int& size);

		int columns()const;
		void setColumns(const int& size);

		real width()const;
		void setWidth(const real& size);

		real height()const;
		void setHeight(const real& size);

		
	private:

		struct Position
		{
			Position() = default;
			int row = 0;
			int column = 0;
		};
		void updateGrid();
		void changeGridSize();
		void updateBodies();
		std::vector<Position> queryCells(const AABB& aabb);
		int m_rows;
		int m_columns;
		real m_width;
		real m_height;

		std::vector<std::list<Body*>> m_gridMap;
		std::map<Body*, std::list<Position>> m_table;

		real m_cellWidth = 0.0f;
		real m_cellHeight = 0.0f;
	};
}

#endif // !PHYSICS_BROADPHASE_GRID_H
