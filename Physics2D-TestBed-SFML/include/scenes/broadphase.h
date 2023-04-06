#ifndef PHYSICS2D_SCENES_BROADPHASE_H
#define PHYSICS2D_SCENES_BROADPHASE_H
#include <random>
#include "physics2d_grid.h"
#include "frame.h"
#include "physics2d_sap.h"

namespace Physics2D
{
	class BroadPhaseFrame : public Frame
	{
	public:
		BroadPhaseFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
		                Tree* tree, UniformGrid* grid, Camera* camera) : Frame(
			"Broadphase", world, maintainer, tree, grid, camera)
		{
		}

		void load() override
		{
			rectangle.set(0.5f, 0.5f);
			circle.setRadius(0.5f);
			capsule.set(1.5f, 0.5f);
			triangle.append({{-1.0f, 1.0f}, {0.0f, -2.0f}, {1.0f, -1.0f}});
			polygon.append({
				{0.0f, 4.0f}, {-3.0f, 3.0f}, {-4.0f, 0.0f}, {-3.0f, -3.0f}, {0, -4.0f},
				{3.0f, -3.0f}, {4.0f, 0.0f}, {3.0f, 3.0f}
			});
			triangle.scale(0.5f);
			polygon.scale(0.1f);


			Shape* shapeArray[5];
			shapeArray[0] = &rectangle;
			shapeArray[1] = &circle;
			shapeArray[2] = &triangle;
			shapeArray[3] = &polygon;
			shapeArray[4] = &capsule;

			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<> dist1(-9.0f, 9.0f);
			std::uniform_int_distribution<> dist2(0, 4);
			std::uniform_real_distribution<> dist3(-Constant::Pi, Constant::Pi);

			for (int i = 0; i < 200; i++)
			{
				Body* body = m_world->createBody();
				body->position().set(dist1(gen), dist1(gen));
				body->setShape(shapeArray[dist2(gen)]);
				body->rotation() = dist3(gen);
				body->setMass(1);
				body->setType(Body::BodyType::Kinematic);

				m_tree->insert(body);
				grid.insert(body);
			}
			for (auto iter = m_world->bodyList().begin(); iter != m_world->bodyList().end(); ++iter)
			{
				bodyList.emplace_back(iter->get());
			}

			//block.set(6.5f, 6.5f);
			//body = m_world->createBody();
			//body->setShape(&block);
			//body->setMass(1);
			//body->setType(Body::BodyType::Kinematic);
			//grid.insert(body)

			//m_tree->insert(body);
		}

		void render(sf::RenderWindow& window) override
		{
			//grid spatial hashing
			grid.updateAll();
			auto pairs = grid.generate();
			sf::Color collisionColor = RenderConstant::Pink;
			sf::Color hitColor = RenderConstant::Blue;
			sf::Color regionColor = RenderConstant::Yellow;
			sf::Color cellColor = sf::Color::Cyan;
			cellColor.a = 155;
			collisionColor.a = 50;
			for (auto&& elem : pairs)
			{
				RenderSFMLImpl::renderBody(window, *m_camera, elem.first, collisionColor);
				RenderSFMLImpl::renderBody(window, *m_camera, elem.second, collisionColor);
			}

			for (auto&& elem : grid.m_cellsToBodies)
			{
				Vector2 topLeft(static_cast<real>(elem.first.x) * grid.cellWidth() - grid.width() * 0.5f,
				                static_cast<real>(elem.first.y) * grid.cellHeight() - grid.height() * 0.5f);
				AABB cell(topLeft, grid.cellWidth(), grid.cellHeight());
				//cell.expand(-0.05f);

				RenderSFMLImpl::renderAABB(window, *m_camera, cell, cellColor);
			}


			//for (auto&& [key, value] : resultBodies)
			//{
			//	RenderSFMLImpl::renderBody(window, *m_camera, value, hitColor);
			//}
			//sweep and prune
			//auto pairs = SweepAndPrune::generate(bodyList);
			//sf::Color collisionColor = RenderConstant::Pink;
			//sf::Color hitColor = RenderConstant::Blue;
			//sf::Color regionColor = RenderConstant::Yellow;
			//for(auto&& elem: pairs)
			//{
			//	RenderSFMLImpl::renderBody(window, *m_camera, elem.first, collisionColor);
			//	RenderSFMLImpl::renderBody(window, *m_camera, elem.second, collisionColor);
			//}

			//AABB queryRegion;
			//queryRegion.width = 8;
			//queryRegion.height = 8;
			//auto resultList = SweepAndPrune::query(bodyList, queryRegion);
			//
			//RenderSFMLImpl::renderAABB(window, *m_camera, queryRegion, regionColor);

			//for(auto& elem: resultList)
			//	RenderSFMLImpl::renderAABB(window, *m_camera, elem->aabb(), hitColor);

			//spatial hash grid
		}

	private:
		UniformGrid grid;
		Body* body;
		Rectangle block;
		Rectangle rectangle;
		Circle circle;
		Polygon polygon;
		Capsule capsule;
		Polygon triangle;
		Container::Vector<Body*> bodyList;
	};
}
#endif
