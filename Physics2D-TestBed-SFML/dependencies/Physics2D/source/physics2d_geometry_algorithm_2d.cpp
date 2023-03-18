#include "physics2d_geometry_algorithm_2d.h"

namespace Physics2D
{
	Container::Vector<Vec2> GeometryAlgorithm2D::Clipper::sutherlandHodgmentPolygonClipping(const Container::Vector<Vec2>& polygon, const Container::Vector<Vec2>& clipRegion)
	{
		Container::Vector<Vec2> result = polygon;

		for (size_t i = 0; i < clipRegion.size() - 1; i++)
		{
			Vec2 clipPoint1 = clipRegion[i];
			Vec2 clipPoint2 = clipRegion[i + 1];
			Vec2 clipDirectionPoint = i + 2 == clipRegion.size() ? clipRegion[1] : clipRegion[i + 2];
			Container::Vector<int8_t> testResults;
			testResults.reserve(polygon.size());

			for (size_t j = 0; j < result.size(); j++)
			{
				bool res = GeometryAlgorithm2D::isPointOnSameSide(clipPoint1, clipPoint2, clipDirectionPoint, result[j]);
				testResults.emplace_back(res ? 1 : -1);
			}
			Container::Vector<Vec2> newPolygon;
			newPolygon.reserve(result.size());

			for (size_t j = 1; j < testResults.size(); j++)
			{
				bool lastInside = testResults[j - 1] == 1 ? true : false;
				bool currentInside = testResults[j] == 1 ? true : false;
				//last inside and current outside
				if (lastInside && !currentInside)
				{
					//push last point
					newPolygon.emplace_back(result[j - 1]);
					//push intersection point
					Vec2 p = GeometryAlgorithm2D::lineIntersection(clipPoint1, clipPoint2, result[j - 1], result[j]);
					newPolygon.emplace_back(p);
				}
				//last outside and current inside
				if (!lastInside && currentInside)
				{
					//push intersection point first
					Vec2 p = GeometryAlgorithm2D::lineIntersection(clipPoint1, clipPoint2, result[j - 1], result[j]);
					newPolygon.emplace_back(p);
				}
				//last outside and current outside
				if (!lastInside && !currentInside)
				{
					//do nothing
				}
				if (lastInside && currentInside)
				{
					//push last vertex
					newPolygon.emplace_back(result[j - 1]);
				}
			}
			result = newPolygon;
			result.emplace_back(result[0]);
		}
		return result;
	}
	bool GeometryAlgorithm2D::isCollinear(const Vec2& a, const Vec2& b, const Vec2& c)
	{
		//triangle area = 0 then collinear
		return realEqual(std::fabs((a - b).cross(a - c)), 0);
	}

	bool GeometryAlgorithm2D::isPointOnSegment(const Vec2& a, const Vec2& b, const Vec2& c)
	{
		return !isCollinear(a, b, c) ? false : fuzzyIsCollinear(a, b, c);
	}

	bool GeometryAlgorithm2D::fuzzyIsPointOnSegment(const Vec2& a, const Vec2& b, const Vec2& c,
	                                                const real& epsilon)
	{
		return fuzzyRealEqual(pointToLineSegment(a, b, c).magnitudeSquare(), epsilon);
	}
	bool GeometryAlgorithm2D::fuzzyIsCollinear(const Vec2& a, const Vec2& b, const Vec2& c)
	{
		return (c.x <= max(a.x, b.x) && c.x >= min(a.x, b.x) &&
			c.y <= max(a.y, b.y) && c.y >= min(a.y, b.y));
	}
	std::optional<Vec2> GeometryAlgorithm2D::lineSegmentIntersection(const Vec2& a, const Vec2& b,
	                                                                    const Vec2& c, const Vec2& d)
	{
		const Vec2 ab = b - a;
		const Vec2 ac = c - a;
		const Vec2 ad = d - a;
		const Vec2 bc = c - b;
		Vec2 ba = a - b;
		const Vec2 bd = d - b;
		const real ab_length = ab.magnitude();

		if (realEqual(ab_length, 0.0))
		{
			if (fuzzyIsCollinear(c, d, a))
				return std::optional(a);
			return std::nullopt;
		}
		const real ab_length_inv = 1 / ab_length;
		const real cc_proj = ab.cross(ac) * ab_length_inv;
		const real dd_proj = ba.cross(bd) * ab_length_inv;
		const real ad_proj = ad.dot(ab.normal());
		const real bc_proj = bc.dot(ba.normal());
		const real cproj_dproj = ab_length - ad_proj - bc_proj;

		if (realEqual(cc_proj, 0.0))
			return std::nullopt;

		const real denominator = (1 + (dd_proj / cc_proj));
		if (realEqual(denominator, 0.0))
			return std::nullopt;

		const real cp = cproj_dproj / denominator;
		const Vec2 bp = ba.normalize() * (bc_proj + cp);
		if (realEqual(bp.magnitude(), 0))
			return std::nullopt;

		Vec2 p = bp + b;

		return (fuzzyIsCollinear(a, b, p) && fuzzyIsCollinear(d, c, p))
			       ? std::optional(p)
			       : std::nullopt;
	}

	Vec2 GeometryAlgorithm2D::lineIntersection(const Vec2& p1, const Vec2& p2, const Vec2& q1,
		const Vec2& q2)
	{
		const real d = (p1.x - p2.x) * (q1.y - q2.y) - (p1.y - p2.y) * (q1.x - q2.x);
		if (realEqual(d, 0))
			return Vec2();
		const real x = ((p1.x * p2.y - p1.y * p2.x) * (q1.x - q2.x) - (q1.x * q2.y - q1.y * q2.x) * (p1.x - p2.x)) / d;
		const real y = ((p1.x * p2.y - p1.y * p2.x) * (q1.y - q2.y) - (q1.x * q2.y - q1.y * q2.x) * (p1.y - p2.y)) / d;
		return Vec2(x, y);
	}

	std::optional<Vec2> GeometryAlgorithm2D::triangleCircumcenter(const Vec2& a, const Vec2& b,
	                                                                 const Vec2& c)
	{
		if (realEqual(triangleArea(a, b, c), 0))
			return std::nullopt;

		//2 * (x2 - x1) * x + 2 * (y2 - y1) y = x2 ^ 2 + y2 ^ 2 - x1 ^ 2 - y1 ^ 2;
		//2 * (x3 - x2) * x + 2 * (y3 - y2) y = x3 ^ 2 + y3 ^ 2 - x2 ^ 2 - y2 ^ 2;
		Mat2 coef_mat{2.0f * (b.x - a.x), 2.0f * (c.x - b.x), 2.0f * (b.y - a.y), 2 * (c.y - b.y)};
		const Vec2 constant{b.magnitudeSquare() - a.magnitudeSquare(), c.magnitudeSquare() - b.magnitudeSquare()};
		return std::optional(coef_mat.invert().multiply(constant));
	}

	std::optional<Vec2> GeometryAlgorithm2D::triangleIncenter(const Vec2& a, const Vec2& b, const Vec2& c)
	{
		if (triangleArea(a, b, c) == 0)
			return std::nullopt;

		const real ab = (b - a).magnitude();
		const real bc = (c - b).magnitude();
		const real ca = (a - c).magnitude();
		Vec2 p = (ab * c + bc * a + ca * b) / (ab + bc + ca);
		return std::optional(p);
	}

	std::optional<std::tuple<Vec2, real>> GeometryAlgorithm2D::calculateCircumcircle(
		const Vec2& a, const Vec2& b, const Vec2& c)
	{
		if (triangleArea(a, b, c) == 0)
			return std::nullopt;
		auto point = triangleCircumcenter(a, b, c);
		real radius = (point.value() - a).magnitude();
		return std::make_tuple(point.value(), radius);
	}

	std::optional<std::tuple<Vec2, real>> GeometryAlgorithm2D::calculateInscribedCircle(
		const Vec2& a, const Vec2& b, const Vec2& c)
	{
		const real area = triangleArea(a, b, c);
		if (area == 0)
			return std::nullopt;

		const real ab = (b - a).magnitude();
		const real bc = (c - b).magnitude();
		const real ca = (a - c).magnitude();
		Vec2 p = (ab * c + bc * a + ca * b) / (ab + bc + ca);
		real radius = 2.0f * area / (ab + bc + ca);
		return std::make_tuple(p, radius);
	}

	bool GeometryAlgorithm2D::isConvexPolygon(const Container::Vector<Vec2>& vertices)
	{
		if (vertices.size() == 4)
			return true;

		for (size_t i = 0; i < vertices.size() - 1; i++)
		{
			Vec2 ab = vertices[i + 1] - vertices[i];
			Vec2 ac = i + 2 != vertices.size() ? vertices[i + 2] - vertices[i] : vertices[1] - vertices[i];
			if (Vec2::crossProduct(ab, ac) < 0)
				return false;
		}
		return true;
	}

	Container::Vector<Vec2> GeometryAlgorithm2D::grahamScan(const Container::Vector<Vec2>& vertices)
	{
		Container::Vector<Vec2> points = vertices;
		Container::Vector<uint32_t> stack;
		std::sort(points.begin(), points.end(), [](const Vec2& a, const Vec2& b)
			{
				if (atan2l(a.y, a.x) != atan2l(b.y, b.x))
					return atan2l(a.y, a.x) < atan2l(b.y, b.x);
				return a.x < b.x;
			});
		uint32_t targetIndex = 0;
		real targetX = points[0].x;
		for (int i = 1; i < points.size(); ++i)
		{
			if (points[i].x < targetX)
			{
				targetIndex = i;
				targetX = points[i].x;
			}
			if (realEqual(points[i].x, targetX))
				if (points[i].y < points[targetIndex].y)
					targetIndex = i;

		}

		stack.emplace_back(targetIndex);
		stack.emplace_back((targetIndex + 1) % points.size());
		uint32_t k = ((targetIndex + 1) % points.size() + 1) % points.size();
		while (true)
		{
			uint32_t i = stack[stack.size() - 2];
			uint32_t j = stack[stack.size() - 1];
			if (j == targetIndex)
				break;

			k %= points.size();

			Vec2 ab = points[j] - points[i];
			Vec2 bc = points[k] - points[j];
			if (ab.cross(bc) < 0) {
				stack.pop_back();
				if (stack.size() < 2)
				{
					stack.emplace_back(k);
					k++;
				}
				continue;
			}
			stack.emplace_back(k);
			k++;
		}
		Container::Vector<Vec2> convex;
		convex.reserve(stack.size());
		for (const auto index : stack)
			convex.emplace_back(points[index]);

		return convex;
	}


	Vec2 GeometryAlgorithm2D::pointToLineSegment(const Vec2& a, const Vec2& b, const Vec2& p)
	{
		//special cases
		if (a == b)
			return {};

		if (isCollinear(a, b, p))
			return p;

		const Vec2 ap = p - a;
		const Vec2 ab_normal = (b - a).normal();
		const Vec2 ap_proj = ab_normal.dot(ap) * ab_normal;
		Vec2 op_proj = a + ap_proj;

		if (fuzzyIsCollinear(a, b, op_proj))
			return op_proj;
		return (p - a).magnitudeSquare() > (p - b).magnitudeSquare() ? b : a;
	}

	Vec2 GeometryAlgorithm2D::shortestLengthPointOfEllipse(const real& a, const real& b, const Vec2& p,
	                                                          const real& epsilon)
	{
		if (realEqual(a, 0) || realEqual(b, 0))
			return {};

		if (realEqual(p.x, 0))
		{
			return p.y > 0
				       ? Vec2{0, b}
				       : Vec2{0, -b};
		}
		if (realEqual(p.y, 0))
		{
			return p.x > 0
				       ? Vec2{a, 0}
				       : Vec2{-a, 0};
		}

		real x_left, x_right;
		Vec2 t0, t1;
		const int sgn = p.y > 0 ? 1 : -1;
		if (p.x < 0)
		{
			x_left = -a;
			x_right = 0;
		}
		else
		{
			x_left = 0;
			x_right = a;
		}
		int iteration = 0;
		while (++iteration)
		{
			real temp_x = (x_left + x_right) * 0.5f;
			real temp_y = sgn * sqrt(pow(b, 2.0f) - pow(b / a, 2.0f) * pow(temp_x, 2.0f));
			Vec2 t0(temp_x, temp_y);
			t0.set(temp_x, temp_y);
			real t1_x = temp_x + 1;
			real t1_y = (pow(b, 2.0f) - pow(b / a, 2.0f) * t1_x * temp_x) / temp_y;
			t1.set(t1_x, t1_y);
			Vec2 t0t1 = t1 - t0;
			Vec2 t0p = p - t0;

			const real result = t0t1.dot(t0p);
			if (std::fabs(result) < epsilon)
				break;

			if (result > 0) // acute angle
				x_left = temp_x;
			else
				x_right = temp_x; //obtuse angle
		}
		return t0;
	}

	Vec2 GeometryAlgorithm2D::triangleCentroid(const Vec2& a1, const Vec2& a2, const Vec2& a3)
	{
		return Vec2(a1 + a2 + a3) / 3.0f;
	}

	real GeometryAlgorithm2D::triangleArea(const Vec2& a1, const Vec2& a2, const Vec2& a3)
	{
		return std::fabs(Vec2::crossProduct(a1 - a2, a1 - a3)) / 2.0f;
	}

	Vec2 GeometryAlgorithm2D::calculateCenter(const Container::Vector<Vec2>& vertices)
	{
		if (vertices.size() >= 4)
		{
			Vec2 pos;
			real area = 0;
			size_t p_a, p_b, p_c;
			p_a = 0, p_b = 0, p_c = 0;
			for (size_t i = 0; i < vertices.size() - 1; i++)
			{
				p_b = i + 1;
				p_c = i + 2;
				if (p_b == vertices.size() - 2)
					break;
				real a = triangleArea(vertices[p_a], vertices[p_b], vertices[p_c]);
				Vec2 p = triangleCentroid(vertices[p_a], vertices[p_b], vertices[p_c]);
				pos += p * a;
				area += a;
			}
			pos /= area;
			return pos;
		}
			return Vec2();
	}

	std::tuple<Vec2, Vec2> GeometryAlgorithm2D::shortestLengthLineSegmentEllipse(
		const real& a, const real& b, const Vec2& p1, const Vec2& p2)
	{
		Vec2 p_line;
		Vec2 p_ellipse;
		if (realEqual(p1.y, p2.y))
		{
			if (!((p1.x > 0 && p2.x > 0) || (p1.x < 0 && p2.x < 0))) // different quadrant
			{
				p_ellipse.set(0, p1.y > 0 ? b : -b);
				p_line.set(0, p1.y);
			}
			else
			{
				p_line.set(std::fabs(p1.x) > std::fabs(p2.x) ? p2.x : p1.x, p1.y);
				p_ellipse = shortestLengthPointOfEllipse(a, b, p_line);
			}
		}
		else if (realEqual(p1.x, p2.x))
		{
			if (!((p1.y > 0 && p2.y > 0) || (p1.y < 0 && p2.y < 0))) // different quadrant
			{
				p_ellipse.set(p1.x > 0 ? a : -a, 0);
				p_line.set(p1.x, 0);
			}
			else
			{
				p_line.set(p1.x, std::fabs(p1.y) > std::fabs(p2.y) ? p2.y : p1.y);
				p_ellipse = shortestLengthPointOfEllipse(a, b, p_line);
			}
		}
		else
		{
			//calculate tangent line
			const real k = (p2.y - p1.y) / (p2.x - p1.x);
			const real k2 = k * k;
			const real a2 = a * a;
			const real b2 = b * b;
			const real f_x2 = (k2 * a2 * a2 / b2) / (1 + a2 * k2 / b2);
			const real f_y2 = b2 - b2 * f_x2 / a2;
			const real f_x = sqrt(f_x2);
			const real f_y = sqrt(f_y2);
			Vec2 f;
			const Vec2 p1p2 = (p2 - p1).normal();

			//Check which quadrant does nearest point fall in
			{
				Vec2 f_arr[4];
				f_arr[0].set(f_x, f_y);
				f_arr[1].set(-f_x, f_y);
				f_arr[2].set(-f_x, -f_y);
				f_arr[3].set(f_x, -f_y);
				real min = Vec2::crossProduct(p1p2, f_arr[0] - p1);
				for (int i = 1; i < 4; i++)
				{
					const real value = Vec2::crossProduct(p1p2, f_arr[i] - p1);
					if (min > value)
					{
						f = f_arr[i];
						min = value;
					}
				}
			}

			const Vec2 p1f = f - p1;
			const Vec2 p1_fp = p1p2 * p1p2.dot(p1f);
			const Vec2 f_proj = p1 + p1_fp;

			if (fuzzyIsCollinear(a, b, f_proj))
			{
				p_ellipse = f;
				p_line = f_proj;
			}
			else
			{
				const Vec2 p1_p = shortestLengthPointOfEllipse(a, b, p1);
				const Vec2 p2_p = shortestLengthPointOfEllipse(a, b, p2);
				if ((p1 - p1_p).magnitudeSquare() > (p2 - p2_p).magnitudeSquare())
				{
					p_ellipse = p2_p;
					p_line = p2;
				}
				else
				{
					p_ellipse = p1_p;
					p_line = p1;
				}
			}
		}
		return std::make_tuple(p_ellipse, p_line);
	}

	std::optional<Vec2> GeometryAlgorithm2D::raycast(const Vec2& p, const Vec2& dir, const Vec2& a,
	                                                    const Vec2& b)
	{
		const real denominator = (p.x - dir.x) * (a.y - b.y) - (p.y - dir.y) * (a.x - b.x);

		if (realEqual(denominator, 0))
			return std::nullopt;

		const real t = ((p.x - a.x) * (a.y - b.y) - (p.y - a.y) * (a.x - b.x)) / denominator;
		const real u = ((dir.x - p.x) * (p.y - a.y) - (dir.y - p.y) * (p.x - a.x)) / denominator;
		if (t >= 0 && u <= 1.0 && u >= 0)
			return std::optional<Vec2>({p.x + t * (dir.x - p.x), p.y + t * (dir.y - p.y)});
		return std::nullopt;
	}

	std::optional<std::pair<Vec2, Vec2>> GeometryAlgorithm2D::raycastAABB(const Vec2& p, const Vec2& dir, const Vec2& topLeft, const Vec2& bottomRight)
	{
		const real xmin = topLeft.x;
		const real ymin = bottomRight.y;
		const real xmax = bottomRight.x;
		const real ymax = topLeft.y;
		real txmin, txmax, tymin, tymax;
		real txenter, txexit, tyenter, tyexit;
		real tenter, texit;
		if(realEqual(dir.x, 0) && !realEqual(dir.y, 0))
		{
			tymin = (ymin - p.y) / dir.y;
			tymax = (ymax - p.y) / dir.y;
			tenter = min(tymin, tymax);
			texit = max(tymin, tymax);
		}
		else if (!realEqual(dir.x, 0) && realEqual(dir.y, 0))
		{

			txmin = (xmin - p.x) / dir.x;
			txmax = (xmax - p.x) / dir.x;
			tenter = min(txmin, txmax);
			texit = max(txmin, txmax);
		}
		else
		{
			txmin = (xmin - p.x) / dir.x;
			txmax = (xmax - p.x) / dir.x;
			tymin = (ymin - p.y) / dir.y;
			tymax = (ymax - p.y) / dir.y;
			txenter = min(txmin, txmax);
			txexit = max(txmin, txmax);
			tyenter = min(tymin, tymax);
			tyexit = max(tymin, tymax);
			tenter = max(txenter, tyenter);
			texit = min(txexit, tyexit);
		}
		if (tenter < 0 && texit < 0)
			return std::nullopt;
		Vec2 enter = p + tenter * dir;
		Vec2 exit = p + texit * dir;

		return std::make_pair(enter, exit);

	}

	bool GeometryAlgorithm2D::isPointOnAABB(const Vec2& p, const Vec2& topLeft, const Vec2& bottomRight)
	{
		return isInRange(p.x, topLeft.x, bottomRight.x) &&
			isInRange(p.y, bottomRight.y, topLeft.y);
	}

	Vec2 GeometryAlgorithm2D::rotate(const Vec2& p, const Vec2& center, const real& angle)
	{
		return Mat2(angle).multiply(p - center) + center;
	}

	Vec2 GeometryAlgorithm2D::calculateEllipseProjectionPoint(const real& a, const real& b, const Vec2& direction)
	{
		Vec2 target;
		if (realEqual(direction.x, 0))
		{
			const int sgn = direction.y < 0 ? -1 : 1;
			target.set(0, sgn * b);
		}
		else if (realEqual(direction.y, 0))
		{
			const int sgn = direction.x < 0 ? -1 : 1;
			target.set(sgn * a, 0);
		}
		else
		{
			const real k = direction.y / direction.x;
			//line offset constant d
			const real a2 = pow(a, 2.0f);
			const real b2 = pow(b, 2.0f);
			const real k2 = pow(k, 2.0f);
			real d = sqrt((a2 + b2 * k2) / k2);
			if (Vec2::dotProduct(Vec2(0, d), direction) < 0)
				d = d * -1;
			const real x1 = k * d - (b2 * k2 * k * d) / (a2 + b2 * k2);
			const real y1 = (b2 * k2 * d) / (a2 + b2 * k2);
			target.set(x1, y1);
		}
		return target;
	}
	Vec2 GeometryAlgorithm2D::calculateCapsuleProjectionPoint(const real& width, const real& height,
	                                                             const Vec2& direction)
	{
		Vec2 target;
		if (width >= height) // Horizontal
		{
			real radius = height / 2.0f;
			real offset = direction.x >= 0 ? width / 2 - radius : radius - width / 2;
			target = direction.normal() * radius;
			target.x += offset;
		}
		else // Vertical
		{
			real radius = width / 2.0f;
			real offset = direction.y >= 0 ? height / 2 - radius : radius - height / 2;
			target = direction.normal() * radius;
			target.y += offset;
		}
		return target;
	}

	Vec2 GeometryAlgorithm2D::calculateSectorProjectionPoint(const real& startRadian, const real& spanRadian,
		const real& radius, const Vec2& direction)
	{
		Vec2 result;
		auto clampRadian = [](const real& radian)
		{
			real result = radian;
			result -= std::floor(result / Constant::TwoPi) * Constant::TwoPi;
			if (result < 0)
				result += Constant::TwoPi;
			return result;
		};

		const real clampStart = clampRadian(startRadian);
		const real clampEnd = clampRadian(startRadian + spanRadian);
		const real originStart = clampRadian(startRadian - Constant::HalfPi);
		const real originEnd = clampRadian(startRadian + spanRadian + Constant::HalfPi);
		
		const real originTheta = direction.theta();
		real theta = clampRadian(originTheta);

		if(originStart > originEnd)
		{
			//does not fall in zero area
			if (!isInRange(theta, originEnd, originStart))
			{
				if(theta > originStart)
					theta = originTheta;
				
				//clamp theta to sector area
				
				result = Mat2(clamp(theta, clampStart, clampEnd)).multiply(Vec2{ 1, 0 }) * radius;
			}
		}
		if(originStart < originEnd)
		{
			if (isInRange(originTheta, originStart, originEnd))
			{
				//clamp theta to sector area
				result = Mat2(clamp(theta, clampStart, clampEnd)).multiply(Vec2{ 1, 0 }) * radius;
			}
		}
		//special case for half circle
		if(fuzzyRealEqual(originStart, originEnd))
		{
			if(!fuzzyRealEqual(theta, originStart))
			{
				if (theta > originStart)
					theta = originTheta;
				if (clampStart > clampEnd)
					result = Mat2(clamp(theta, clampStart - Constant::TwoPi, clampEnd)).multiply(Vec2{ 1, 0 }) * radius;
				else
					result = Mat2(clamp(theta, clampStart, clampEnd)).multiply(Vec2{ 1, 0 }) * radius;
			}
		}
		return result;
	}

	bool GeometryAlgorithm2D::triangleContainsOrigin(const Vec2& a, const Vec2& b, const Vec2& c)
	{
		real ra = (b - a).cross(-a);
		real rb = (c - b).cross(-b);
		real rc = (a - c).cross(-c);
		return sameSign(ra, rb, rc);
	}
	bool GeometryAlgorithm2D::isPointOnSameSide(const Vec2& edgePoint1, const Vec2& edgePoint2, const Vec2& refPoint, const Vec2 targetPoint)
	{
		Vec2 u = edgePoint2 - edgePoint1;
		Vec2 v = refPoint - edgePoint1;
		Vec2 w = targetPoint - edgePoint1;
		//same side or on the edge
		return sameSign(u.cross(v), u.cross(w));
	}
}
