//
// Created by petr on 4/26/16.
//

#include "gui.h"

using crl::logger;
CShape redLine("red", "red", 1, 0);

crl::gui::CCanvasBase &operator<<(crl::gui::CCanvasBase &base, opendubins::Dubins &dubins) {
	//INFO("gui dubins write");
	Arc firstArc = dubins.getFirstArc();
	Arc secondArc = dubins.getSecondArc();

	base << firstArc;
	base << secondArc;

	if ( dubins.isCCC ) {
		Arc carc = dubins.getCenterArc();
		base << carc;
	} else {
		Line line = dubins.getCenter();
		base << line;
	}
	return base;
}

crl::gui::CCanvasBase &operator<<(crl::gui::CCanvasBase &base, opendubins::Line &line) {
	base << canvas::LINE;
	base << line.p1.x << line.p1.y;
	base << line.p2.x << line.p2.y;
	base << canvas::END;
	return base;
}

crl::gui::CCanvasBase &operator<<(crl::gui::CCanvasBase &base, opendubins::Arc &arc) {
	if ( fabs(arc.angle) > TOLERANCE ) {

		base << canvas::ARC << Fill(false);
		Point p1 = arc.getCenter();
		double startAngle = -M_PI / 2 - arc.state.ang;
		if ( arc.angle >= 0 ) {
			startAngle += M_PI;
		}

		double endAngle = -M_PI / 2 - arc.state.ang - arc.angle;
		if ( arc.angle >= 0 ) {
			endAngle += M_PI;
		}

		if ( arc.angle >= 0 ) {
			std::swap(startAngle, endAngle);
		}

		while ( endAngle <= startAngle ) {
			endAngle += 2 * M_PI;
		}

		base << p1.x << p1.y << arc.radius << startAngle << endAngle;
	}
	base << canvas::END;
	return base;
}

crl::gui::CCanvasBase &operator<<(crl::gui::CCanvasBase &base, opendubins::Point& point) {
	base << canvas::POINT << point.x << point.y << canvas::END;
	return base;
}

crl::gui::CCanvasBase &operator<<(crl::gui::CCanvasBase &base, opendubins::Circle &circle) {
	CShape redLine("blue", "blue", 1, 0);
	base << canvas::ARC << redLine << Fill(false); // << FillColor("black") << Width(2);
	double startAngle = 0;
	double endAngle = 2 * M_PI;
	base << circle.center.x << circle.center.y << circle.radius << startAngle << endAngle;
	base << canvas::END;

	//base << canvas::POINT << circleLine << circle.center.x << circle.center.y << canvas::END;
	return base;
}

crl::gui::CCanvasBase & drawObstacle(crl::gui::CCanvasBase &base, Obstacle &obstacle,
		std::vector<MapPoint> &map_points, CShape * shape) {
	CShape line = redLine;
	if ( shape != NULL ) {
		line = *shape;
	}
	base << canvas::POLYGON << line;
	for ( int verid = 0 ; verid < obstacle.point_indexes.size() ; ++verid ) {
		//INFO("obstacle point "<< map_points[obstacles[obstid].point_indexes[verid]].x <<" "<< map_points[obstacles[obstid].point_indexes[verid]].y);
		base << map_points[obstacle.point_indexes[verid]].x << map_points[obstacle.point_indexes[verid]].y;
	}
	base << canvas::END;
	base.redraw();
	return base;
}

crl::gui::CCanvasBase & drawObstacle(crl::gui::CCanvasBase &base, MeshObject* obstacle,
		std::vector<MapPoint> &map_points, CShape * shape) {
	CShape line = redLine;
	if ( shape != NULL ) {
		line = *shape;
	}
	Point3D* vertices = obstacle->getVertices();
	for ( int var = 0 ; var < obstacle->getFacesNum() ; ++var ) {
		Face face = obstacle->getFaces()[var];
		base << canvas::POLYGON << line;
		base << vertices[face.id1].x << vertices[face.id1].y;
		base << vertices[face.id2].x << vertices[face.id2].y;
		base << vertices[face.id3].x << vertices[face.id3].y;
		base << canvas::END;
	}
	base.redraw();
	return base;
}

