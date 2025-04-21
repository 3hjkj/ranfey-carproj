#pragma once
#include "vector"
#include"string"
#include"glm/glm.hpp"
using namespace std;
namespace OpenDrive
{
	enum EnOpcode
	{
		ODR_OPCODE_NONE = 0,
		ODR_OPCODE_HEADER,
		ODR_OPCODE_ROAD_HEADER,
		ODR_OPCODE_ROAD_LINK,
		ODR_OPCODE_ROAD_TYPE,
		ODR_OPCODE_GEO_HEADER,
		ODR_OPCODE_GEO_LINE,
		ODR_OPCODE_GEO_SPIRAL,
		ODR_OPCODE_GEO_ARC,
		ODR_OPCODE_Elevation,
		ODR_OPCODE_LANE_SECTION, // 10
		ODR_OPCODE_LANE,
		ODR_OPCODE_LANE_LINK,
		ODR_OPCODE_LANE_WIDTH,
		ODR_OPCODE_LANE_MATERIAL,
		ODR_OPCODE_LANE_VISIBILITY,
		ODR_OPCODE_SIGNAL,
		ODR_OPCODE_LANE_VALIDITY,
		ODR_OPCODE_SIGNAL_DEPEND,
		ODR_OPCODE_CONTROLLER,
		ODR_OPCODE_CONTROL_ENTRY, // 20
		ODR_OPCODE_JUNCTION_HEADER,
		ODR_OPCODE_JUNCTION_LINK,
		ODR_OPCODE_JUNCTION_PRIORITY,
		ODR_OPCODE_OBJECT,
		ODR_OPCODE_USER_DATA,
		ODR_OPCODE_JUNCTION_LANE_LINK,
		ODR_OPCODE_CROSSFALL,
		ODR_OPCODE_JUNCTION_CONTROL,
		ODR_OPCODE_LANE_ROAD_MARK,
		ODR_OPCODE_PREDECESSOR, // 30
		ODR_OPCODE_SUCCESSOR,
		ODR_OPCODE_LANES,
		ODR_OPCODE_LANES_LEFT,
		ODR_OPCODE_LANES_CENTER,
		ODR_OPCODE_LANES_RIGHT,
		ODR_OPCODE_PLANVIEW,
		ODR_OPCODE_ELEV_PROFILE,
		ODR_OPCODE_LATERAL_PROFILE,
		ODR_OPCODE_OBJECTS,
		ODR_OPCODE_SIGNALS,  // 40
		ODR_OPCODE_OPENDRIVE,
		ODR_OPCODE_SUPERElevation,
		ODR_OPCODE_GEO_POLY,
		ODR_OPCODE_LANE_SPEED,
		ODR_OPCODE_LANE_ACCESS,
		ODR_OPCODE_LANE_HEIGHT,
		ODR_OPCODE_CORNER_INERTIAL,
		ODR_OPCODE_CORNER_ROAD,
		ODR_OPCODE_CORNER_RELATIVE,
		ODR_OPCODE_TUNNEL,  // 50
		ODR_OPCODE_BRIDGE,
		ODR_OPCODE_SIGNAL_REFERENCE,
		ODR_OPCODE_OBJECT_OUTLINE,
		ODR_OPCODE_SURFACE,
		ODR_OPCODE_SURFACE_CRG,
		ODR_OPCODE_LANE_OFFSET,
		ODR_OPCODE_GENERIC_NODE,
		ODR_OPCODE_CORNER_LOCAL,
		ODR_OPCODE_REPEAT,
		ODR_OPCODE_GEO_PARAM_POLY,  // 60
		ODR_OPCODE_LANE_BORDER,
		ODR_OPCODE_ROAD_SPEED,
		ODR_OPCODE_GEO_REFERENCE,
		ODR_OPCODE_LATERAL_SHAPE,
		ODR_OPCODE_JUNCTION_GROUP,
		ODR_OPCODE_JUNCTION_REFERENCE,
		ODR_OPCODE_ROAD_MARK_TYPE,
		ODR_OPCODE_ROAD_MARK_LINE,
		ODR_OPCODE_PARKING_SPACE,
		ODR_OPCODE_PARKING_SPACE_MARKING,
		ODR_OPCODE_NEIGHBOR,
		ODR_OPCODE_TRAFFIC_OBJECT = 2000,
		ODR_OPCODE_RAILROAD_SWITCH
	};

	// GEOMETRY TYPE
	enum EnGeometryType {
		ODR_GEO_TYPE_LINE = 0,
		ODR_GEO_TYPE_SPIRAL,
		ODR_GEO_TYPE_ARC,
		ODR_GEO_TYPE_POLY3
	};

	// ROAD MARK
	enum EnRoadMark {
		ODR_ROAD_MARK_NONE = 0,
		ODR_ROAD_MARK_SOLID,
		ODR_ROAD_MARK_SOLID_BOLD,
		ODR_ROAD_MARK_BROKEN,
		ODR_ROAD_MARK_SOLID_DOUBLE,
		ODR_ROAD_MARK_BROKEN_BOLD,
		ODR_ROAD_MARK_SOLID_YELLOW,
		ODR_ROAD_MARK_SOLID_DOUBLE_YELLOW
	};

	enum EnRoadMarkType {
		ODR_ROAD_MARK_TYPE_NONE = 0,
		ODR_ROAD_MARK_TYPE_SOLID,
		ODR_ROAD_MARK_TYPE_BROKEN,
		ODR_ROAD_MARK_TYPE_SOLID_SOLID,
		ODR_ROAD_MARK_TYPE_SOLID_BROKEN,
		ODR_ROAD_MARK_TYPE_BROKEN_SOLID,
		ODR_ROAD_MARK_TYPE_CURB
	};

	enum EnRoadMarkWeight {
		ODR_ROAD_MARK_WEIGHT_NONE = 0,
		ODR_ROAD_MARK_WEIGHT_STANDARD,
		ODR_ROAD_MARK_WEIGHT_BOLD
	};

	enum EnRoadMarkColor {
		ODR_ROAD_MARK_COLOR_NONE = 0,
		ODR_ROAD_MARK_COLOR_STANDARD,
		ODR_ROAD_MARK_COLOR_YELLOW,
		ODR_ROAD_MARK_COLOR_RED,
		ODR_ROAD_MARK_COLOR_WHITE
	};

	// LANE TYPE
	enum EnLaneType {
		ODR_LANE_TYPE_NONE = 0,
		ODR_LANE_TYPE_DRIVING,
		ODR_LANE_TYPE_STOP,
		ODR_LANE_TYPE_SHOULDER,
		ODR_LANE_TYPE_BIKING,
		ODR_LANE_TYPE_SIDEWALK,
		ODR_LANE_TYPE_BORDER,
		ODR_LANE_TYPE_RESTRICTED,
		ODR_LANE_TYPE_PARKING,
		ODR_LANE_TYPE_MWY_ENTRY,
		ODR_LANE_TYPE_MWY_EXIT,
		ODR_LANE_TYPE_SPECIAL1,
		ODR_LANE_TYPE_SPECIAL2,
		ODR_LANE_TYPE_SPECIAL3,
		ODR_LANE_TYPE_SPECIAL4,
		ODR_LANE_TYPE_DRIVING_ROADWORKS,
		ODR_LANE_TYPE_TRAM,
		ODR_LANE_TYPE_RAIL
	};

	struct OdrInfo
	{
		char* _roadID = NULL;
		char* _sectionID = NULL;
		char* _laneID = NULL;
		double _s = -1.0;
		double _t = -1.0;
	};
	struct Position
	{
		double _s = 0.0;
		double _x = 0.0;
		double _y = 0.0;
		double _direction = 0.0; 
		double _Elevation = 0.0;
		double _crossfall = 0.0;
	};

	struct RoadWidth
	{
		double _s = 0.0;
		double _roadWith = 0.0;
	};

	struct TurningAttr
	{
		double _s = 0.0;
		double _turnRad = 0.0;
	};
	
	struct NearestDistance
	{
		double _leftLaneDistance = 0.0;
		double _rightLaneDistance = 0.0;

		double _leftRoadDistance = 0.0;
		double _rightRoadDistance = 0.0;
	};
	//roadMark
	struct LaneMark
	{
		EnRoadMarkColor _color;
		EnRoadMarkType _type;
		EnRoadMarkWeight _weight;
		double _sOffset;
	};
	struct NearestRoadMark
	{
		LaneMark *_leftRoadMark = NULL;
		LaneMark *_rightRoadMark = NULL;
	};
	struct LanePostion
	{
		double _s = 0.0;
		double _x = 0.0;
		double _y = 0.0;
	};

	struct PosXY
	{
		double _x = 0.0;
		double _y = 0.0;
	};


	struct RoadProperty
	{
		bool _bRight = true;
		vector<int> _vecLaneID;
		EnLaneType _laneType = ODR_LANE_TYPE_NONE;
	};

	struct GeoCoordinatePoint
	{
		double _s = 0.0;
		double _x = 0.0;
		double _y = 0.0;
		double _z = 0.0;
		double _hdg = 0.0;
	};
	struct RoadPolygon
	{
		vector<Position> _vecPos;
	};

	struct SignalPos
	{
		double _s = 0.0;
		double _t = 0.0; 
		double _zOFF = 0.0; 
		double _x = 0.0;//相对于车的x坐标;
		double _y = 0.0;//相对于车的y坐标;
	};

	struct LaneRelation
	{
		char* _roadID = NULL;
		char* _sectionID = NULL;
		char* _laneID = NULL;
	};

	// LANE TYPE
	enum RoadDrivingAttr {
		ODR_ROAD_TYPE_NONE =0,
		ODR_ROAD_TYPE_STraightLine,
		ODR_ROAD_TYPE_UTurning,
		ODR_ROAD_TYPE_LTurning,
		ODR_ROAD_TYPE_RTurning
	};
	struct LaneGeoCoordinage
	{
		int indx = 0;
		vector<GeoCoordinatePoint> vecGeoLane;

	};
	
}
