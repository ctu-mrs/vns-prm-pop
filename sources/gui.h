//
// Created by petr on 4/26/16.
//

#pragma once

#include "crl/gui/canvas.h"
#include "crl/logging.h"
#include "opendubins/line.h"
#include "opendubins/point.h"
#include "opendubins/dubins.h"
#include "opendubins/arc.h"
#include "opendubins/circle.h"
#include "heuristic_types.h"
#include "mesh_object.h"

using namespace crl::gui;
using namespace opendubins;

//crl::gui::CCanvasBase &operator<<(crl::gui::CCanvasBase &base, opendubins::Shape& shape);
crl::gui::CCanvasBase &operator<<(crl::gui::CCanvasBase &base, opendubins::Dubins& dubins);
crl::gui::CCanvasBase &operator<<(crl::gui::CCanvasBase &base, opendubins::Line& line);
crl::gui::CCanvasBase &operator<<(crl::gui::CCanvasBase &base, opendubins::Arc& arc);
crl::gui::CCanvasBase &operator<<(crl::gui::CCanvasBase &base, opendubins::Point& point);
crl::gui::CCanvasBase &operator<<(crl::gui::CCanvasBase &base,  opendubins::Circle &circle) ;
crl::gui::CCanvasBase & drawObstacle(crl::gui::CCanvasBase &base, Obstacle &obstacle, std::vector<MapPoint> &map_points,CShape * shape = NULL);
crl::gui::CCanvasBase & drawObstacle(crl::gui::CCanvasBase &base, MeshObject* obstacle, std::vector<MapPoint> &map_points, CShape * shape= NULL);

