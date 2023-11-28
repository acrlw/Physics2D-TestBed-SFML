#include "physics2d_contact.h"
#include "physics2d_contact.h"


namespace Physics2D
{
	void ContactMaintainer::clearAll()
	{
		m_contactTable.clear();
	}

	void ContactMaintainer::solveVelocity(real dt)
	{
		for (auto&& elem : m_contactTable)
		{
			if (elem.second.empty())
				continue;

			//solve friction first
			for (auto&& ccp : elem.second)
			{
				if (!ccp.active)
					continue;

				auto& vcp = ccp.vcp;
				vcp.va = ccp.bodyA->velocity() + Vector2::crossProduct(ccp.bodyA->angularVelocity(), vcp.ra);
				vcp.vb = ccp.bodyB->velocity() + Vector2::crossProduct(ccp.bodyB->angularVelocity(), vcp.rb);
				Vector2 dv = vcp.va - vcp.vb;

				real jvt = vcp.tangent.dot(dv);
				real lambda_t = vcp.effectiveMassTangent * -jvt;

				real maxFriction = ccp.friction * vcp.accumulatedNormalImpulse;
				real newImpulse = Math::clamp(vcp.accumulatedTangentImpulse + lambda_t, -maxFriction, maxFriction);
				lambda_t = newImpulse - vcp.accumulatedTangentImpulse;
				vcp.accumulatedTangentImpulse = newImpulse;

				Vector2 impulse_t = lambda_t * vcp.tangent;

				ccp.bodyA->applyImpulse(impulse_t, vcp.ra);
				ccp.bodyB->applyImpulse(-impulse_t, vcp.rb);
			}

			if(m_velocityBlockSolver && elem.second.size() == 2)
			{
				//start block solver
				if (!elem.second[0].active || !elem.second[1].active)
					return;

				auto& ccp = elem.second[0];

				Body* bodyA = elem.second[0].bodyA;
				Body* bodyB = elem.second[0].bodyB;

				auto& vcp1 = elem.second[0].vcp;
				auto& vcp2 = elem.second[1].vcp;

				Vector2 wa1 = Vector2::crossProduct(bodyA->angularVelocity(), vcp1.ra);
				Vector2 wb1 = Vector2::crossProduct(bodyB->angularVelocity(), vcp1.rb);
				vcp1.va = bodyA->velocity() + wa1;
				vcp1.vb = bodyB->velocity() + wb1;

				Vector2 wa2 = Vector2::crossProduct(bodyA->angularVelocity(), vcp2.ra);
				Vector2 wb2 = Vector2::crossProduct(bodyB->angularVelocity(), vcp2.rb);
				vcp2.va = bodyA->velocity() + wa2;
				vcp2.vb = bodyB->velocity() + wb2;

				Vector2 dv1 = vcp1.va - vcp1.vb;
				Vector2 dv2 = vcp2.va - vcp2.vb;

				Vector2 normal = vcp1.normal;

				real jv1 = normal.dot(dv1 - vcp1.velocityBias);
				real jv2 = normal.dot(dv2 - vcp2.velocityBias);



				//build quadratic programming:

				//min_{x} 0.5 * x^T * A * x + b^T * x
				//differentiate: f(x) = Ax + b
				//it is better that Ax + b = 0

				//for(;;)
				//{
				//	//1. b_1 < 0 && b_2 < 0
				//	if(nb.x < 0.0f && nb.y < 0.0f)
				//	{
				//		nx = ccp.normalMass.multiply(-nb);
				//		if(nx.x < 0.0f || nx.y < 0.0f)
				//		{
				//			//hit error point
				//			int a = 0;
				//		}
				//		break;
				//	}
				//	//2. b_1 < 0 && b_2 > 0
				//	if(nb.x < 0.0f && nb.y >= 0.0f)
				//	{
				//		nx.x = vcp1.effectiveMassNormal * -nb.x;
				//		nx.y = 0.0f;
				//		if (nx.x < 0.0f || nx.y < 0.0f)
				//		{
				//			//hit error point
				//			int a = 0;
				//		}
				//		break;
				//	}
				//	//3. b_1 > 0 && b_2 < 0
				//	if(nb.x >= 0.0f && nb.y < 0.0f)
				//	{
				//		nx.x = 0.0f;
				//		nx.y = vcp2.effectiveMassNormal * -nb.y;
				//		if (nx.x < 0.0f || nx.y < 0.0f)
				//		{
				//			//hit error point
				//			int a = 0;
				//		}
				//		break;
				//	}
				//	//4. b_1 > 0 && b_2 > 0
				//	if(nb.x >= 0.0f && nb.y >= 0.0f)
				//	{
				//		nx.clear();
				//		break;
				//	}
				//	break;
				//}
				////clamp or projected
				//nx.x = Math::max(nx.x, 0.0f);
				//nx.y = Math::max(nx.y, 0.0f);

				//LCP: y = Ax + b, x >= 0, y >= 0, xy = 0
				//A: ccp.K
				//A^{-1}: ccp.normalMass
				//b: [jv1, jv2]
				//x: [lambda1, lambda2]
				//nx: next x
				//d: delta x
				Matrix2x2 A = ccp.k;
				Vector2 b(jv1, jv2);
				Vector2 x(vcp1.accumulatedNormalImpulse, vcp2.accumulatedNormalImpulse);
				Vector2 nx;
				Vector2 d;

				b = b - A.multiply(x);

				for(;;)
				{
					//1. b_1 < 0 && b_2 < 0
					nx = ccp.normalMass.multiply(-b);
					if(nx.x >= 0.0f && nx.y >= 0.0f)
						break;
					
					//2. b_1 < 0 && b_2 > 0
					nx.x = vcp1.effectiveMassNormal * -b.x;
					nx.y = 0.0f;
					jv1 = 0.0f;
					jv2 = A.e12() * nx.x + b.y;
					if(nx.x >= 0.0f && jv2 >= 0.0f)
						break;
					
					//3. b_1 > 0 && b_2 < 0
					nx.x = 0.0f;
					nx.y = -vcp2.effectiveMassNormal * b.y;
					jv1 = A.e21() * nx.y + b.x;
					jv2 = 0.0f;
					if(nx.y >= 0.0f && jv1 >= 0.0f)
						break;
					
					//4. b_1 > 0 && b_2 > 0
					nx.clear();
					jv1 = b.x;
					jv2 = b.y;
					if(jv1 >= 0.0f && jv2 >= 0.0f)
						break;
					
					//hit the unknown cases
					int a = 0;
					break;
				}


				d = nx - x;

				real& lambda_1 = d.x;
				real& lambda_2 = d.y;

				Vector2 impulse_1 = lambda_1 * normal;
				Vector2 impulse_2 = lambda_2 * normal;

				bodyA->applyImpulse(impulse_1, vcp1.ra);
				bodyB->applyImpulse(-impulse_1, vcp1.rb);

				bodyA->applyImpulse(impulse_2, vcp2.ra);
				bodyB->applyImpulse(-impulse_2, vcp2.rb);

				vcp1.accumulatedNormalImpulse = nx.x;
				vcp2.accumulatedNormalImpulse = nx.y;

				//use fixed point iteration(projected gradient method)

				

				//Vector2 x;

				//const real alpha = 0.1f;

				//const int maxIteration = 1000;



				//for(int i = 0;i < maxIteration; ++i)
				//{
				//	Vector2 dx = alpha * (A.multiply(x) + b);
				//	assert(!std::isinf(dx.x));
				//	assert(!std::isinf(dx.y));
				//	assert(!std::isinf(x.x));
				//	assert(!std::isinf(x.y));
				//	Vector2 nx = x - dx;

				//	//clamp nx >= 0
				//	nx.x = Math::max(nx.x, 0.0f);
				//	nx.y = Math::max(nx.y, 0.0f);
				//	//alert nan

				//	//if change is too small then break
				//	if ((nx - x).lengthSquare() < 1e-6f)
				//		break;

				//	x = nx;
				//}


				//real& lambda_1 = x.x;
				//real& lambda_2 = x.y;

				//Vector2 impulse_1 = lambda_1 * vcp1.normal;
				//Vector2 impulse_2 = lambda_2 * vcp2.normal;

				//apply impulse to bodyA, bodyB


			}
			else
			{
				for (auto&& ccp : elem.second)
				{
					if (!ccp.active)
						continue;

					auto& vcp = ccp.vcp;

					Vector2 wa = Vector2::crossProduct(ccp.bodyA->angularVelocity(), vcp.ra);
					Vector2 wb = Vector2::crossProduct(ccp.bodyB->angularVelocity(), vcp.rb);
					vcp.va = ccp.bodyA->velocity() + wa;
					vcp.vb = ccp.bodyB->velocity() + wb;

					Vector2 dv = vcp.va - vcp.vb;
					real jv = vcp.normal.dot(dv - vcp.velocityBias);
					real lambda_n = vcp.effectiveMassNormal * -jv;
					real oldImpulse = vcp.accumulatedNormalImpulse;
					vcp.accumulatedNormalImpulse = Math::max(oldImpulse + lambda_n, 0);
					lambda_n = vcp.accumulatedNormalImpulse - oldImpulse;

					Vector2 impulse_n = lambda_n * vcp.normal;

					ccp.bodyA->applyImpulse(impulse_n, vcp.ra);
					ccp.bodyB->applyImpulse(-impulse_n, vcp.rb);
				}
			}

		}
	}

	void ContactMaintainer::solveRestitution(real dt)
	{
		for (auto&& elem : m_contactTable)
		{
			if (elem.second.empty())
				continue;
			for (auto&& ccp : elem.second)
			{
				if (!ccp.active)
					continue;

				auto& vcp = ccp.vcp;

				if(vcp.restitution == 0.0f || vcp.normal.isOrigin() && vcp.relativeVelocity < -1e-8f)
					continue;

				Vector2 wa = Vector2::crossProduct(ccp.bodyA->angularVelocity(), vcp.ra);
				Vector2 wb = Vector2::crossProduct(ccp.bodyB->angularVelocity(), vcp.rb);
				vcp.va = ccp.bodyA->velocity() + wa;
				vcp.vb = ccp.bodyB->velocity() + wb;

				Vector2 dv = vcp.va - vcp.vb;
				real jv = vcp.normal.dot(dv);

				real lambda_r = vcp.effectiveMassNormal * -(jv + vcp.restitution * vcp.relativeVelocity);

				Vector2 impulse_r = lambda_r * vcp.normal;

				ccp.bodyA->applyImpulse(impulse_r, vcp.ra);
				ccp.bodyB->applyImpulse(-impulse_r, vcp.rb);
			}
		}
	}

	void ContactMaintainer::solvePosition(real dt)
	{
		for (auto&& elem : m_contactTable)
		{
			if (elem.second.empty() || !elem.second[0].active)
				continue;
			for (;;)
			{
				if (m_positionBlockSolver && elem.second.size() == 2)
				{
					//start block solver
					auto&& vcp1 = elem.second[0].vcp;
					auto&& vcp2 = elem.second[1].vcp;
					Body* bodyA = elem.second[0].bodyA;
					Body* bodyB = elem.second[0].bodyB;

					Vector2 pa1 = bodyA->toWorldPoint(vcp1.localA);
					Vector2 pb1 = bodyB->toWorldPoint(vcp1.localB);
					Vector2 pa2 = bodyA->toWorldPoint(vcp2.localA);
					Vector2 pb2 = bodyB->toWorldPoint(vcp2.localB);

					Vector2 ra1 = pa1 - bodyA->position();
					Vector2 rb1 = pb1 - bodyB->position();
					Vector2 ra2 = pa2 - bodyA->position();
					Vector2 rb2 = pb2 - bodyB->position();

					Vector2 c1 = pb1 - pa1;
					Vector2 c2 = pb2 - pa2;

					real bias1 = Math::max(m_biasFactor * (c1.dot(vcp1.normal) - m_maxPenetration), 0.0f);
					real bias2 = Math::max(m_biasFactor * (c2.dot(vcp2.normal) - m_maxPenetration), 0.0f);

					const real bias = Math::min(bias1, bias2);

					bias1 = -bias;
					bias2 = -bias;

					const real im_a = bodyA->inverseMass();
					const real im_b = bodyB->inverseMass();
					const real ii_a = bodyA->inverseInertia();
					const real ii_b = bodyB->inverseInertia();

					real rn1A = ra1.cross(vcp1.normal);
					real rn1B = rb1.cross(vcp1.normal);
					real rn2A = ra2.cross(vcp2.normal);
					real rn2B = rb2.cross(vcp2.normal);

					real k11 = im_a + ii_a * rn1A * rn1A + im_b + ii_b * rn1B * rn1B;
					real k12 = im_a + ii_a * rn1A * rn2A + im_b + ii_b * rn1B * rn2B;
					real k22 = im_a + ii_a * rn2A * rn2A + im_b + ii_b * rn2B * rn2B;

					real determinant = (k11 * k22 - k12 * k12);
					real d1 = k11 * k11;
					real d2 = 1000.0f * (k11 * k22 - k12 * k12);
					bool conditioner = k11 * k11 < 1000.0f * (k11 * k22 - k12 * k12);

					//numerical stability check to ensure invertible matrix
					Matrix2x2 A;
					Matrix2x2 invA;
					if (conditioner)
					{
						A.set(k11, k12, k12, k22);
						invA = A;
						invA.invert();
					}
					else
						break;


					Vector2 b(bias1, bias2);
					Vector2 d;

					for (;;)
					{
						//1. b_1 < 0 && b_2 < 0
						Vector2 x = invA.multiply(-b);
						if (x.x >= 0.0f && x.y >= 0.0f)
						{
							d = x;
							break;
						}
						//2. b_1 < 0 && b_2 > 0
						x.x = -b.x / k11;
						x.y = 0.0f;
						bias2 = A.e21() * x.x + b.y;
						if (x.x >= 0.0f && bias2 >= 0.0f)
						{
							d = x;
							break;
						}

						//3. b_1 > 0 && b_2 < 0
						x.x = 0.0f;
						x.y = -b.y / k22;
						bias1 = A.e12() * x.y + b.x;
						if (x.y >= 0.0f && bias1 >= 0.0f)
						{
							d = x;
							break;
						}

						//4. b_1 > 0 && b_2 > 0
						//d = zero

						break;
					}

					Vector2 impulse1 = vcp1.normal * d.x;
					Vector2 impulse2 = vcp2.normal * d.y;

					bodyA->position() += bodyA->inverseMass() * impulse1;
					bodyA->rotation() += bodyA->inverseInertia() * ra1.cross(impulse1);

					bodyB->position() -= bodyB->inverseMass() * impulse1;
					bodyB->rotation() -= bodyB->inverseInertia() * rb1.cross(impulse1);

					bodyA->position() += bodyA->inverseMass() * impulse2;
					bodyA->rotation() += bodyA->inverseInertia() * ra2.cross(impulse2);

					bodyB->position() -= bodyB->inverseMass() * impulse2;
					bodyB->rotation() -= bodyB->inverseInertia() * rb2.cross(impulse2);
				}
				break;
			}
			for (auto&& ccp : elem.second)
			{
				auto&& vcp = ccp.vcp;
				Body* bodyA = ccp.bodyA;
				Body* bodyB = ccp.bodyB;
				Vector2 pa = bodyA->toWorldPoint(vcp.localA);
				Vector2 pb = bodyB->toWorldPoint(vcp.localB);
				Vector2 ra = pa - bodyA->position();
				Vector2 rb = pb - bodyB->position();
				Vector2 c = pb - pa;

				const real bias = Math::max(m_biasFactor * (c.dot(vcp.normal) - m_maxPenetration), 0.0f);

				const real im_a = bodyA->inverseMass();
				const real im_b = bodyB->inverseMass();
				const real ii_a = bodyA->inverseInertia();
				const real ii_b = bodyB->inverseInertia();

				const real rn_a = ra.cross(vcp.normal);
				const real rn_b = rb.cross(vcp.normal);

				const real kNormal = im_a + ii_a * rn_a * rn_a +
					im_b + ii_b * rn_b * rn_b;

				vcp.effectiveMassNormal = realEqual(kNormal, 0.0f) ? 0 : 1.0f / kNormal;

				real lambda = vcp.effectiveMassNormal * bias;
				lambda = Math::max(lambda, 0);

				Vector2 impulse = lambda * vcp.normal;

				bodyA->position() += bodyA->inverseMass() * impulse;
				bodyA->rotation() += bodyA->inverseInertia() * ra.cross(impulse);

				bodyB->position() -= bodyB->inverseMass() * impulse;
				bodyB->rotation() -= bodyB->inverseInertia() * rb.cross(impulse);
			}
		}
	}

	void ContactMaintainer::add(const Collision& collision)
	{
		const Body* bodyA = collision.bodyA;
		const Body* bodyB = collision.bodyB;
		const bool isRoundA = bodyA->shape()->type() == ShapeType::Circle || bodyA->shape()->type() == ShapeType::Ellipse;
		const bool isRoundB = bodyB->shape()->type() == ShapeType::Circle || bodyB->shape()->type() == ShapeType::Ellipse;
		const auto relation = Body::BodyPair::generateBodyPairID(collision.bodyA, collision.bodyB);
		auto& contactList = m_contactTable[relation];
		uint32_t i = 0;

		for (; i < collision.contactList.count; i += 2)
		{
			VertexPair elem;
			elem.pointA = collision.contactList.points[i];
			elem.pointB = collision.contactList.points[i + 1];

			bool existed = false;
			Vector2 localA = bodyA->toLocalPoint(elem.pointA);
			Vector2 localB = bodyB->toLocalPoint(elem.pointB);
			for (auto& contact : contactList)
			{
				const bool isPointA = localA.fuzzyEqual(contact.localA, Constant::TrignometryEpsilon);
				const bool isPointB = localB.fuzzyEqual(contact.localB, Constant::TrignometryEpsilon);
				if (isPointA || isPointB || isRoundA || isRoundB)
				{
					//satisfy the condition, give the old accumulated value to new value
					contact.localA = localA;
					contact.localB = localB;
					prepare(contact, elem, collision);
					existed = true;
					break;
				}
			}
			if (existed)
				continue;

			//no eligible contact, push new contact points
			ContactConstraintPoint ccp;
			ccp.localA = localA;
			ccp.localB = localB;
			ccp.relation = relation;
			prepare(ccp, elem, collision);
			contactList.emplace_back(ccp);
		}
		//remove inactive contact
		//std::erase_if(contactList, [](const ContactConstraintPoint& ccp)
		//	{
		//		return !ccp.active;
		//	});

		
		//assert(collision.contactList.count / 2 == contactList.size());
		if(m_velocityBlockSolver && contactList.size() == 2)
		{
			//prepare block solver
			auto& vcp1 = contactList[0].vcp;
			auto& vcp2 = contactList[1].vcp;

			const real im_a = collision.bodyA->inverseMass();
			const real im_b = collision.bodyB->inverseMass();
			const real ii_a = collision.bodyA->inverseInertia();
			const real ii_b = collision.bodyB->inverseInertia();

			real rn1A = vcp1.ra.cross(collision.normal);
			real rn1B = vcp1.rb.cross(collision.normal);
			real rn2A = vcp2.ra.cross(collision.normal);
			real rn2B = vcp2.rb.cross(collision.normal);

			real k11 = im_a + ii_a * rn1A * rn1A + im_b + ii_b * rn1B * rn1B;
			real k12 = im_a + ii_a * rn1A * rn2A + im_b + ii_b * rn1B * rn2B;
			real k22 = im_a + ii_a * rn2A * rn2A + im_b + ii_b * rn2B * rn2B;

			real determinant = (k11 * k22 - k12 * k12);
			real d1 = k11 * k11;
			real d2 = 1000.0f * (k11 * k22 - k12 * k12);
			bool conditioner = k11 * k11 < 1000.0f * (k11 * k22 - k12 * k12);

			//numerical stability check to ensure invertible matrix
			if(conditioner)
			{
				Matrix2x2 k(k11, k12, k12, k22);
				contactList[0].k = k;
				contactList[1].k = k;
				k.invert();
				contactList[0].normalMass = k;
				contactList[1].normalMass = k;
			}
		}
	}

	void ContactMaintainer::clearInactivePoints()
	{

		for (auto&& iter : m_contactTable)
		{
			auto& contactList = iter.second;
			std::erase_if(contactList, [](const ContactConstraintPoint& ccp)
			{
				return !ccp.active;
			});
		}
		std::erase_if(m_contactTable, [](const auto& item)
		{
			const auto& [key, value] = item;
			return value.empty();
		});

	}

	void ContactMaintainer::deactivateAllPoints()
	{
		for (auto iter = m_contactTable.begin(); iter != m_contactTable.end(); ++iter)
		{
			if (iter->second.empty() || !iter->second[0].active)
				continue;

			for (auto& ccp : iter->second)
				ccp.active = false;
		}

	}

	void ContactMaintainer::prepare(ContactConstraintPoint& ccp, const VertexPair& pair, const Collision& collision)
	{
		ccp.bodyA = collision.bodyA;
		ccp.bodyB = collision.bodyB;
		ccp.active = true;

		ccp.friction = Math::sqrt(ccp.bodyA->friction() * ccp.bodyB->friction());

		VelocityConstraintPoint& vcp = ccp.vcp;
		vcp.localA = collision.bodyA->toLocalPoint(pair.pointA);
		vcp.localB = collision.bodyB->toLocalPoint(pair.pointB);

		vcp.ra = pair.pointA - collision.bodyA->position();
		vcp.rb = pair.pointB - collision.bodyB->position();

		vcp.normal = collision.normal;
		vcp.tangent = vcp.normal.perpendicular();

		const real im_a = collision.bodyA->inverseMass();
		const real im_b = collision.bodyB->inverseMass();
		const real ii_a = collision.bodyA->inverseInertia();
		const real ii_b = collision.bodyB->inverseInertia();

		const real rn_a = vcp.ra.cross(vcp.normal);
		const real rn_b = vcp.rb.cross(vcp.normal);

		const real rt_a = vcp.ra.cross(vcp.tangent);
		const real rt_b = vcp.rb.cross(vcp.tangent);

		const real kNormal = im_a + ii_a * rn_a * rn_a +
			im_b + ii_b * rn_b * rn_b;

		const real kTangent = im_a + ii_a * rt_a * rt_a +
			im_b + ii_b * rt_b * rt_b;

		vcp.effectiveMassNormal = realEqual(kNormal, 0.0f) ? 0 : 1.0f / kNormal;
		vcp.effectiveMassTangent = realEqual(kTangent, 0.0f) ? 0 : 1.0f / kTangent;

		//vcp.bias = 0;
		vcp.restitution = Math::min(ccp.bodyA->restitution(), ccp.bodyB->restitution());
		vcp.penetration = collision.penetration;

		//vcp.velocityBias = -vcp.restitution * (vcp.va - vcp.vb);
		vcp.velocityBias = 0;

		//accumulate inherited impulse
		Vector2 impulse = vcp.accumulatedNormalImpulse * vcp.normal + vcp.accumulatedTangentImpulse * vcp.tangent;
		//Vector2 impulse;
		if (m_warmStart)
		{
			ccp.bodyA->applyImpulse(impulse, vcp.ra);
			ccp.bodyB->applyImpulse(-impulse, vcp.rb);
		}

		Vector2 wa = Vector2::crossProduct(ccp.bodyA->angularVelocity(), vcp.ra);
		Vector2 wb = Vector2::crossProduct(ccp.bodyB->angularVelocity(), vcp.rb);
		vcp.va = ccp.bodyA->velocity() + wa;
		vcp.vb = ccp.bodyB->velocity() + wb;

		vcp.relativeVelocity = vcp.normal.dot(vcp.va - vcp.vb);
	}
}
