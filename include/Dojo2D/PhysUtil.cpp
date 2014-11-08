#include "stdafx.h"

#include "PhysUtil.h"

std::vector<b2PolygonShape> Phys::decomposeConvex(const std::vector<Vector>& points) {
	b2Polygon wholePolygon;

	//warning: b2Polygon acquires x and y and frees with delete
	wholePolygon.x = new float32[points.size()];
	wholePolygon.y = new float32[points.size()];
	wholePolygon.nVertices = points.size();

	for (size_t i = 0; i < points.size(); ++i)
	{
		wholePolygon.x[i] = points[i].x;
		wholePolygon.y[i] = points[i].y;
	}

	//cannot decompose, return an empty vector
	if (!wholePolygon.IsSimple())
		return {};

	const int MAX_PIECES = 100;

	b2Polygon polys[MAX_PIECES];

	int n = DecomposeConvex(&wholePolygon, polys, MAX_PIECES);

	std::vector<b2PolygonShape> processedPolys;
	processedPolys.reserve(n);

	//use AddTo and then extract the polyshape from the fixture!
	b2FixtureDef temp;
	for (int i = 0; i < n; ++i)
	{
		polys[i].AddTo(temp);

		processedPolys.emplace_back(*(b2PolygonShape*)temp.shape);
	}

	return processedPolys;
}
