/************************************************************************/
// project : ImageRender

// author : San Jiang, China University of GeoSciences,
//          School of Computer Science

// e-mail: jiangsan@cug.edu.cn

// date : 2020-10-11

/************************************************************************/

#ifndef IMAGE_CAPTURER_H
#define IMAGE_CAPTURER_H

#include <osg/Geode>

#include "irt/irtSpatialGeometry.hpp"
#include "irt/irtFileNameUtil.hpp"
#include "SaveTransformMatrixComputeBoundingboxVisitor.h"
// #include "DrawableRecorder.h"

class CapturingPosAttUpdater;

class ImageCapturer
{
	friend class CapturingPosAttUpdater;

public:
	typedef irtMath::irtSpatialGeometry::BoundingBox2Df GridTileExtent;

	typedef std::vector<osg::ref_ptr<osg::Image>> ImageContainer;

	struct CapturingCondition
	{
		GridTileExtent			_tile_extent;//grid left up corner
		std::string _point_id;
		osg::Vec2 _pos_lat_lon;
		osg::Vec2i				_capturing_img_size;
		float					_z_lift;
		std::string				_output_folder_path;
		std::string				_output_img_ext;

		// old
		// CapturingCondition() :_z_lift(0.0f){}
		// CapturingCondition(const GridTileExtent& ext, const osg::Vec2i& cap_size, float z_lift, const std::string& output_folder_path, const std::string output_img_ext) :
		// 	_tile_extent(ext), _capturing_img_size(cap_size), _z_lift(z_lift), _output_folder_path(output_folder_path), _output_img_ext(output_img_ext)
		// {
		// 	_output_folder_path = irtFileNameUtil::makePath(_output_folder_path.c_str());
		// 	if (_output_img_ext.at(0) != '.') _output_img_ext = "." + _output_img_ext;
		// }
		// CapturingCondition(const CapturingCondition& other) :_tile_extent(other._tile_extent), _capturing_img_size(other._capturing_img_size), _z_lift(other._z_lift), _output_folder_path(other._output_folder_path), _output_img_ext(other._output_img_ext){}
		// CapturingCondition& operator= (const CapturingCondition& other){ _tile_extent = other._tile_extent; _capturing_img_size = other._capturing_img_size; _z_lift = other._z_lift; _output_folder_path = other._output_folder_path; _output_img_ext = other._output_img_ext; return *this; }
		// bool valid()const{ return _tile_extent.valid() && _capturing_img_size.x() > 0 && _capturing_img_size.y() > 0 && _z_lift >= 0.0; }
		// std::string gen_output_img_name(const std::string viewNumForDebug = "")const
		// {
		// 	std::string center_x_str = irtFileNameUtil::figureToString<double>(_tile_extent.center().x(), 9);
		// 	std::string center_y_str = irtFileNameUtil::figureToString<double>(_tile_extent.center().y(), 9);
		// 	return _output_folder_path + center_x_str + "_" + center_y_str + viewNumForDebug + _output_img_ext;
		// }

		// new
		CapturingCondition() :_z_lift(0.0f){}
		CapturingCondition(const std::string point_id, const osg::Vec2& pos_lat_lon, const osg::Vec2i& cap_size, float z_lift, const std::string& output_folder_path, const std::string output_img_ext) :
			_point_id(point_id),_pos_lat_lon(pos_lat_lon), _capturing_img_size(cap_size), _z_lift(z_lift), _output_folder_path(output_folder_path), _output_img_ext(output_img_ext)
		{
			_output_folder_path = irtFileNameUtil::makePath(_output_folder_path.c_str());
			if (_output_img_ext.at(0) != '.') _output_img_ext = "." + _output_img_ext;
		}
		CapturingCondition(const osg::Vec2& pos_lat_lon, const osg::Vec2i& cap_size, float z_lift, const std::string& output_folder_path, const std::string output_img_ext) :
			_pos_lat_lon(pos_lat_lon), _capturing_img_size(cap_size), _z_lift(z_lift), _output_folder_path(output_folder_path), _output_img_ext(output_img_ext)
		{
			_output_folder_path = irtFileNameUtil::makePath(_output_folder_path.c_str());
			if (_output_img_ext.at(0) != '.') _output_img_ext = "." + _output_img_ext;
		}
		CapturingCondition(const CapturingCondition& other) :_pos_lat_lon(other._pos_lat_lon), _capturing_img_size(other._capturing_img_size), _z_lift(other._z_lift), _output_folder_path(other._output_folder_path), _output_img_ext(other._output_img_ext){}
		CapturingCondition& operator= (const CapturingCondition& other){ _pos_lat_lon = other._pos_lat_lon; _capturing_img_size = other._capturing_img_size; _z_lift = other._z_lift; _output_folder_path = other._output_folder_path; _output_img_ext = other._output_img_ext; return *this; }
		// bool valid()const{ return _tile_extent.valid() && _capturing_img_size.x() > 0 && _capturing_img_size.y() > 0 && _z_lift >= 0.0; }
		std::string gen_output_img_name(const std::string viewNumForDebug = "")const
		{
			std::string center_x_str = irtFileNameUtil::figureToString<double>(_pos_lat_lon.x(), 9);
			std::string center_y_str = irtFileNameUtil::figureToString<double>(_pos_lat_lon.y(), 9);
			// 
			return _output_folder_path +  _point_id + "_" + center_x_str + "_" + center_y_str + viewNumForDebug + _output_img_ext;
		}

		
	};

public:
	inline ImageCapturer() :_cap_cond(CapturingCondition()), _view_num(4){}

	inline ImageCapturer(const CapturingCondition& cap_cond, osg::Node* terrain_node, osg::Geode* shpfile_node, int view_num) : _cap_cond(cap_cond), _terrain_node(terrain_node), _shpfile_node(shpfile_node), _view_num(view_num){
		SaveTransformMatrixComputeBoundingboxVisitor cbv;
		//TerrainTechnique��ImageCapturer���캯���д���ComputeBoundsVisitor
		//�ı��������б���ʼ��,��Ӧ�ı任����Ҳ���ʼ��
		_terrain_node->accept(cbv);
		_terrain_node_bb_info._terrain_boundingbox = cbv.getBoundingBox();
		_terrain_node_bb_info._terrain_technique_transform_mat = cbv.getLastTransformMatrix();
	}

	inline ImageCapturer(const CapturingCondition& cap_cond, osg::Node* terrain_node, int view_num, float image_headig) : _cap_cond(cap_cond), _terrain_node(terrain_node), _view_num(view_num), _image_headig(image_headig){

		SaveTransformMatrixComputeBoundingboxVisitor cbv;

		//TerrainTechnique��ImageCapturer���캯���д���ComputeBoundsVisitor
		//�ı��������б���ʼ��,��Ӧ�ı任����Ҳ���ʼ��
		_terrain_node->accept(cbv);
		_terrain_node_bb_info._terrain_boundingbox = cbv.getBoundingBox();
		_terrain_node_bb_info._terrain_technique_transform_mat = cbv.getLastTransformMatrix();
	}

	// inline ImageCapturer(osg::Node* terrain_node, int view_num) : _terrain_node(terrain_node), _view_num(view_num){
	// 	std::cout<<"1 "<<std::endl;
	// 	SaveTransformMatrixComputeBoundingboxVisitor cbv;
	// 	std::cout<<"2 "<<std::endl;
	// 	//TerrainTechnique��ImageCapturer���캯���д���ComputeBoundsVisitor
	// 	//�ı��������б���ʼ��,��Ӧ�ı任����Ҳ���ʼ��
	// 	_terrain_node->accept(cbv);
	// 	std::cout<<"3 "<<std::endl;
	// 	_terrain_node_bb_info._terrain_boundingbox = cbv.getBoundingBox();
	// 	std::cout<<"4 "<<std::endl;
	// 	_terrain_node_bb_info._terrain_technique_transform_mat = cbv.getLastTransformMatrix();
	// 	std::cout<<"5 "<<std::endl;
	// }

	// void SetCapturCondition(const CapturingCondition& cap_cond);
	// {
	// 	_cap_cond = cap_cond;
	// 	return;
	// }

	virtual inline ~ImageCapturer(){}

private:
	struct HeightFieldInfo
	{
		float			_z_min;
		float			_z_max;

		HeightFieldInfo() :_z_min(FLT_MAX), _z_max(-FLT_MAX){}

		HeightFieldInfo(const HeightFieldInfo& o) :_z_min(o._z_min), _z_max(o._z_max){}

		void update(float z)
		{
			if (_z_min > z) _z_min = z;
			if (_z_max < z) _z_max = z;
		}

		float center()const{ return (_z_max + _z_min) / 2.0; }

		HeightFieldInfo& operator= (const HeightFieldInfo& o)
		{
			_z_min = o._z_min;
			_z_max = o._z_max;
			return *this;
		}
	};

	HeightFieldInfo computeHeightFieldInfoWithinThisTile()const;

	bool is_camera_pos_within_shpfile();

	void cofig_rtt_camera(osg::Camera* rtt, osg::Camera* master)const;

	osg::Matrixd createPerspectiveProjection(int expected_tex_width, int expected_tex_height, osg::Camera* master, int cap_img_num)const;

	osg::Vec3d computeViewDirection(int viewDirection);

	void set_capturing_location_attitude(osg::Camera* rtt, int viewDirection);

	void simulate();//int cap_num

	bool save_capturing_img()const;
public:
	virtual bool capture();

private:
	const CapturingCondition&				_cap_cond;

	osg::ref_ptr<osg::Node>					_terrain_node;

	struct  TerrainNodeBoundingboxInfo{
		osg::Matrixd							_terrain_technique_transform_mat;
		osg::BoundingBox						_terrain_boundingbox;//�ð�Χ���Ǿ���_terrain_technique_transform_mat���ú�İ�Χ��
	};

	TerrainNodeBoundingboxInfo					_terrain_node_bb_info;

	osg::ref_ptr<osg::Geode>				_shpfile_node;

	osg::Vec3								_camera_eye_pos;

	ImageContainer							_img_container;

	int										_view_num;
	float                                _image_headig;
};

#endif // IMAGE_CAPTURER_H
