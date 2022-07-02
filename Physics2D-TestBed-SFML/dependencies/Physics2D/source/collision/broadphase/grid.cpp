#include "../../../include/collision/broadphase/grid.h"

namespace Physics2D
{
	UniformGrid::UniformGrid(const real& width = 100.0f, const real& height = 100.0f, const int rows = 100, const int columns = 100)
		:m_width(width), m_height(height), m_rows(rows), m_columns(columns)
	{
        updateGrid();
	}

    std::vector<std::pair<Body*, Body*>> UniformGrid::generate()
    {
        std::vector<std::pair<Body*, Body*>> result;
        return result;
    }

    std::vector<Body*> UniformGrid::raycast(const Vector2& p, const Vector2& d)
    {
        std::vector<Body*> result;
        return result;
    }

    void UniformGrid::update(Body* body)
    {
        assert(body != nullptr);
    }
    void UniformGrid::insert(Body* body)
    {
        assert(body != nullptr);
    }

    void UniformGrid::remove(Body* body)
    {
        assert(body != nullptr);
        auto iter = m_table.find(body);
        if (iter == m_table.end())
            return;
    }

    int UniformGrid::rows() const
    {
        return m_rows;
    }

    void UniformGrid::setRows(const int& size)
    {
        m_rows = size;
        updateGrid();
    }

    int UniformGrid::columns() const
    {
        return m_columns;
    }

    void UniformGrid::setColumns(const int& size)
    {
        m_columns = size;
        updateGrid();
    }

    real UniformGrid::width() const
    {
        return m_width;
    }

    void UniformGrid::setWidth(const real& size)
    {
        m_width = size;
        updateGrid();
    }

    real UniformGrid::height() const
    {
        return m_height;
    }

    void UniformGrid::setHeight(const real& size)
    {
        m_height = size;
        updateGrid();
    }

    void UniformGrid::updateGrid()
    {
        changeGridSize();
        updateBodies();
    }

    void UniformGrid::changeGridSize()
    {
        assert(m_columns != 0 && m_rows != 0);
        m_cellWidth = m_width / real(m_columns);
        m_cellHeight = m_height / real(m_rows);
    }

    void UniformGrid::updateBodies()
    {

    }

    std::vector<UniformGrid::Position> UniformGrid::queryCells(const AABB& aabb)
    {
        std::vector<UniformGrid::Position> cells;
        //locate x axis

        real xMin = aabb.minimumX();
        real xMax = aabb.maximumX();

        //locate y axis

        real yMin = aabb.minimumY();
        real yMax = aabb.maximumY();
        return cells;

    }
    
}
