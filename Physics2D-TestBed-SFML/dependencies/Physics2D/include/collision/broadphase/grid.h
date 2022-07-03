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
		UniformGrid(const real& width = 100.0f, const real& height = 100.0f, const uint32_t rows = 100, const uint32_t columns = 100);
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

		struct Position
		{
			Position() = default;
			uint32_t x = 0;
			uint32_t y = 0;
			bool operator<(const Position& rhs)const
			{
				if (x < rhs.x)
					return true;
				if (x == rhs.x)
					return y < rhs.y;
				return false;
			}
		};
		std::vector<Position> queryCells(const AABB& aabb);

		real cellHeight()const;
		real cellWidth()const;
	private:

		void updateGrid();
		void changeGridSize();
		void updateBodies();
		real m_width;
		real m_height;
		uint32_t m_rows;
		uint32_t m_columns;

		std::vector<std::list<Body*>> m_gridMap;
		std::map<Body*, std::list<Position>> m_table;

		real m_cellWidth = 0.0f;
		real m_cellHeight = 0.0f;
	};
}

#endif // !PHYSICS_BROADPHASE_GRID_H
