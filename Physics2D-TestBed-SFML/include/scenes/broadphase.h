#ifndef PHYSICS2D_SCENES_BROADPHASE_H
#define PHYSICS2D_SCENES_BROADPHASE_H
#include <random>

#include "./include/frame.h"
#include "dependencies/Physics2D/include/collision/broadphase/sap.h"
namespace Physics2D
{
	class BroadPhaseFrame : public Frame
	{
	public:
		BroadPhaseFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Camera* camera) : Frame("Broad Phase", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
			rectangle.set(0.5f, 0.5f);
			circle.setRadius(0.5f);
			capsule.set(1.5f, 0.5f);
			triangle.append({ {-1.0f, 1.0f},{0.0f, -2.0f},{1.0f, -1.0f},{-1.0f, 1.0f} });
			polygon.append({ {0.0f, 4.0f},{-3.0f, 3.0f},{-4.0f, 0.0f},{-3.0f, -3.0f},{0, -4.0f},
			{3.0f, -3.0f}, {4.0f, 0.0f }, {3.0f, 3.0f },{0.0f, 4.0f } });
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
				body->setType(Body::BodyType::Static);

				m_tree->insert(body);
			}
			for (auto&& elem : m_world->bodyList())
				bodyList.emplace_back(elem.get());
		}
		void render(sf::RenderWindow& window) override
		{
			auto pairs = SweepAndPrune::generate(bodyList);
			sf::Color collisionColor = RenderConstant::materialPink;
			sf::Color hitColor = RenderConstant::materialBlue;
			sf::Color regionColor = RenderConstant::materialYellow;
			//for(auto&& elem: pairs)
			//{
			//	RenderSFMLImpl::renderBody(window, *m_camera, elem.first, collisionColor);
			//	RenderSFMLImpl::renderBody(window, *m_camera, elem.second, collisionColor);
			//}

			AABB queryRegion;
			queryRegion.width = 8;
			queryRegion.height = 8;
			auto resultList = SweepAndPrune::query(bodyList, queryRegion);
			
			RenderSFMLImpl::renderAABB(window, *m_camera, queryRegion, regionColor);

			for(auto& elem: resultList)
				RenderSFMLImpl::renderAABB(window, *m_camera, elem->aabb(), hitColor);

		}
	private:
		Rectangle rectangle;
		Circle circle;
		Polygon polygon;
		Capsule capsule;
		Polygon triangle;
		std::vector<Body*> bodyList;
	};
}
#endif