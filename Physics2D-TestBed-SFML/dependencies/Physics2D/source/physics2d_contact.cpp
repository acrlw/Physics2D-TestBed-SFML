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

			for(auto&& ccp: elem.second)
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

			for (auto&& ccp : elem.second)
			{
				if(!ccp.active)
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

	bool ContactMaintainer::solvePosition(real dt)
	{

		real maxPenetration = 0.0f;
		for (auto&& elem : m_contactTable)
		{
			if (elem.second.empty())
				continue;
			Body* bodyA = elem.second[0].bodyA;
			Body* bodyB = elem.second[0].bodyB;

			const real im_a = bodyA->inverseMass();
			const real im_b = bodyB->inverseMass();
			const real ii_a = bodyA->inverseInertia();
			const real ii_b = bodyB->inverseInertia();

			Vector2 normal = elem.second[0].vcp.normal;
			for (auto&& ccp : elem.second)
			{
				if (!ccp.active)
					continue;

				auto&& vcp = ccp.vcp;

				Vector2 pa = bodyA->toWorldPoint(vcp.contactLocalA);
				Vector2 pb = bodyB->toWorldPoint(vcp.contactLocalB);

				Vector2 ra = pa - bodyA->position();
				Vector2 rb = pb - bodyB->position();
				Vector2 c = pb - pa;

				real projected = c.dot(normal);
				

				real penetration = vcp.penetration - projected;

				maxPenetration = Math::max(maxPenetration, penetration);

				const real bias =  Math::clamp(m_biasFactor * (penetration - m_maxPenetration), 0.0f, 0.2f);

				const real rn_a = ra.cross(normal);
				const real rn_b = rb.cross(normal);

				const real kNormal = im_a + ii_a * rn_a * rn_a +
					im_b + ii_b * rn_b * rn_b;

				const real effectiveMassNormal = realEqual(kNormal, 0.0f) ? 0 : 1.0f / kNormal;

				real lambda = effectiveMassNormal * bias;

				Vector2 impulse = lambda * normal;

				//[DEBUG]
				//vcp.positionCorrectionImpulse = impulse;

				Vector2 dp = bodyA->inverseMass() * impulse;
				real dr = bodyA->inverseInertia() * ra.cross(impulse);
				bodyA->position() += dp;
				bodyA->rotation() += dr;

				dp = bodyB->inverseMass() * impulse;
				dr = bodyB->inverseInertia() * rb.cross(impulse);
				bodyB->position() -= dp;
				bodyB->rotation() -= dr;

			}

		}
		return maxPenetration < 3 * m_maxPenetration;
	}

	void ContactMaintainer::updateContact(const Collision& collision)
	{
		const Body* bodyA = collision.bodyA;
		const Body* bodyB = collision.bodyB;
		const auto relation = Body::BodyPair::generateBodyPairID(collision.bodyA, collision.bodyB);

		auto& contactList = m_contactTable[relation];
		
		for(uint8_t i = 0, j = 0; i < collision.contactList.count; i += 2, ++j)
		{
			VertexPair elem;
			elem.pointA = collision.contactList.points[i];
			elem.pointB = collision.contactList.points[i + 1];

			bool existed = false;
			Vector2 localA = bodyA->toLocalPoint(elem.pointA);
			Vector2 localB = bodyB->toLocalPoint(elem.pointB);
			for (auto& contact : contactList)
			{
				const bool isPointA = localA.fuzzyEqual(contact.collisionLocalA, 1e-5f);
				const bool isPointB = localB.fuzzyEqual(contact.collisionLocalB, 1e-5f);
				
				if (isPointA || isPointB)
				{
					//satisfy the condition, transmit the old accumulated value to new value
					contact.collisionLocalA = localA;
					contact.collisionLocalB = localB;
					prepareContact(contact, elem, collision, collision.contactList.penetration[j]);
					existed = true;
					break;
				}
				
			}
			if (existed)
				continue;
			//no eligible contact, push new contact points
			ContactConstraintPoint ccp;
			ccp.collisionLocalA = localA;
			ccp.collisionLocalB = localB;
			ccp.relation = relation;
			prepareContact(ccp, elem, collision, collision.contactList.penetration[j]);
			contactList.emplace_back(ccp);
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
				auto const& [key, value] = item;
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

	void ContactMaintainer::prepareContact(ContactConstraintPoint& ccp, const VertexPair& pair, const Collision& collision, const real& penetration)
	{
		ccp.bodyA = collision.bodyA;
		ccp.bodyB = collision.bodyB;
		ccp.active = true;

		ccp.friction = Math::sqrt(ccp.bodyA->friction() * ccp.bodyB->friction());

		VelocityConstraintPoint& vcp = ccp.vcp;

		//magic lerp
		const Vector2 contact = Vector2::lerp(pair.pointA, pair.pointB, m_contactLerpFactor);
		

		vcp.contactLocalA = collision.bodyA->toLocalPoint(contact);
		vcp.contactLocalB = collision.bodyB->toLocalPoint(contact);

		vcp.ra = contact - collision.bodyA->position();
		vcp.rb = contact - collision.bodyB->position();

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
		vcp.penetration = penetration;

		Vector2 wa = Vector2::crossProduct(ccp.bodyA->angularVelocity(), vcp.ra);
		Vector2 wb = Vector2::crossProduct(ccp.bodyB->angularVelocity(), vcp.rb);
		vcp.va = ccp.bodyA->velocity() + wa;
		vcp.vb = ccp.bodyB->velocity() + wb;

		vcp.velocityBias = -vcp.restitution * (vcp.va - vcp.vb);

		//accumulate inherited impulse
		Vector2 impulse = vcp.accumulatedNormalImpulse * vcp.normal + vcp.accumulatedTangentImpulse * vcp.tangent;
		//Vector2 impulse;


		ccp.bodyA->applyImpulse(impulse, vcp.ra);
		ccp.bodyB->applyImpulse(-impulse, vcp.rb);
		


	}
}
