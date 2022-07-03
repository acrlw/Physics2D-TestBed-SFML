#include "../../../include/collision/broadphase/grid.h"

namespace Physics2D
{
	UniformGrid::UniformGrid(const real& width, const real& height, const uint32_t rows, const uint32_t columns)
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
        real halfWidth = m_width * 0.5f;
        real halfHeight = m_width * 0.5f;
        real xMin = Math::clamp(aabb.minimumX(), -halfWidth, halfWidth);
        real xMax = Math::clamp(aabb.maximumX(), -halfWidth, halfWidth);
        real lowerXIndex = std::floor((xMin + halfWidth) / m_cellWidth);
        real upperXIndex = std::floor((halfWidth + xMax) / m_cellWidth);
        //locate y axis

        real yMin = Math::clamp(aabb.minimumY(), -halfHeight, halfHeight);
        real yMax = Math::clamp(aabb.maximumY(), -halfHeight, halfHeight);
        real lowerYIndex = std::ceil((yMin + halfHeight) / m_cellHeight);
        real upperYIndex = std::ceil((halfHeight + yMax) / m_cellHeight);
        if (realEqual(lowerXIndex, upperXIndex) || realEqual(lowerYIndex, upperYIndex))
            return cells;

        for(real i = lowerXIndex; i <= upperXIndex; i += m_cellWidth)
        {
            for (real j = lowerYIndex; j <= upperYIndex; j += m_cellHeight)
            {
                cells.emplace_back(Position{ static_cast<uint32_t>(i + halfWidth), static_cast<uint32_t>(j + halfHeight) });
            }
        }

        return cells;

    }

    real UniformGrid::cellHeight() const
    {
        return m_cellHeight;
    }

    real UniformGrid::cellWidth() const
    {
        return m_cellWidth;
    }
}
