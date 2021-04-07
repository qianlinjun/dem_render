//#ifdef _MSC_VER
//#include <Windows.h>
//#include <WinBase.h>
//#include <assert.h>
//#endif

#include <osg/Texture2D>
#include <osgGA/CameraManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/Renderer>
#include <osgDB/WriteFile>
#include <osgDB/DatabasePager>

#ifdef _MSC_VER
#include <osgViewer/api/Win32/GraphicsWindowWin32>
#else
#include <osgViewer/api/X11/GraphicsWindowX11>
#endif

#include "ImageCapturer.h"
#include "PageLODLoadingWatchman.h"
// #include "CoordinateSysLib/coordinate_sys_util.h"


osgDB::DatabasePager* g_dp = 0;
bool g_need_to_change_camera_pos_att = false;


// void ImageCapturer::SetCapturCondition(const ImageCapturer::CapturingCondition& cap_cond)
// {
// 	_cap_cond = cap_cond;//this->
// 	return;
// }

// ImageCapturer::HeightFieldInfo ImageCapturer::computeHeightFieldInfoWithinThisTile() const
// {
// 	HeightFieldInfo z_min_max;
// 	osg::ref_ptr<DrawableRecorder> dr = new DrawableRecorder;
// 	_terrain_node->accept(*dr);
// 	DrawableRecorder::InternalGeomList& geoms = dr->getInternalGeomList();
// 	osg::Vec3 wolrd;
// 	for (int i = 0; i < geoms.size(); i++)
// 	{
// 		osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>(geoms[i]->getVertexArray());
// 		for (unsigned int vi = 0; vi < verts->size(); vi++)
// 		{
// 			//transform local to world
// 			wolrd = verts->at(vi) * _terrain_node_bb_info._terrain_technique_transform_mat;
// 			if (_cap_cond._tile_extent.contains(osg::Vec2(wolrd.x(), wolrd.y()))) z_min_max.update(wolrd.z());
// 		}
// 	}

// 	return z_min_max;
// }

bool getHeightAt(osg::Node* terrainNode, float x, float y, float& h)
{
	osg::BoundingSphere bs = terrainNode->getBound();
	osg::Vec3 center = bs.center();
	// std::cout<<"bs.radius"<<bs.radius()<<"bs.center"<<center.x()<<" "<<center.y()<<" "<<center.z()<<std::endl;
	osg::Vec3 start(x, y, bs.radius());
	osg::Vec3 end(x, y, -bs.radius());

	osg::ref_ptr<osgUtil::LineSegmentIntersector> lsi = new osgUtil::LineSegmentIntersector(start, end);
	lsi->setIntersectionLimit(osgUtil::Intersector::LIMIT_NEAREST);
	osgUtil::IntersectionVisitor iv(lsi);
	terrainNode->accept(iv);

	osgUtil::LineSegmentIntersector::Intersections& results = lsi->getIntersections();
	if (!results.empty())
	{
		const osgUtil::LineSegmentIntersector::Intersection& firstHit = *results.begin();
		osg::Vec3d hit = firstHit.getWorldIntersectPoint();

		h = hit.z();
		return true;
	}

	return false;
}

bool ImageCapturer::is_camera_pos_within_shpfile()
{
	osg::Vec2 tile_center = _cap_cond._tile_extent.center();
	int num = _shpfile_node->getNumDrawables();
	for (int i = 0; i < num; i++)
	{
		osg::Geometry* geom = dynamic_cast<osg::Geometry*>(_shpfile_node->getDrawable(i));


		for (int corner = 0; corner < 4; corner++)
		{
			if (!irtMath::irtSpatialGeometry::isPointInPolygon3D<osg::Vec3, osg::Vec3Array>(osg::Vec3(_cap_cond._tile_extent.corner(corner).x(), _cap_cond._tile_extent.corner(corner).y(), 0.0), geom))
			{
				return false;
			}
		}

		float center_height = 0.0;
		bool getFlag = getHeightAt(_terrain_node, tile_center.x(), tile_center.y(), center_height);
		if(getFlag){
			std::cout<<"get height scussced"<<std::endl;
		}else{
			std::cout<<"get height failed"<<std::endl;
		}
		//HeightFieldInfo hfi = computeHeightFieldInfoWithinThisTile();
		//float z_mid = hfi.center();
		//if (center_height < z_mid)
		//center_height = z_mid;
		//center_height =  center_height + hfi.center();
		_camera_eye_pos.set(tile_center.x(), tile_center.y(), center_height + _cap_cond._z_lift);
		return true;
	}

	return false;
}

void hideRenderConsole(const osgViewer::Viewer* viewer)
{
	const osg::GraphicsContext* gc = viewer->getCamera()->getGraphicsContext();
	if (gc)
	{
#ifdef _MSC_VER
		const osgViewer::GraphicsWindowWin32* gw32 = dynamic_cast<const osgViewer::GraphicsWindowWin32*>(gc);
		HWND hwnd = gw32->getHWND();
		::ShowWindow(hwnd, SW_HIDE);
#else
		const osgViewer::GraphicsWindowX11* gw11 = dynamic_cast<const osgViewer::GraphicsWindowX11*>(gc);
#endif
	}
}

osg::Matrixd ImageCapturer::createPerspectiveProjection(int expected_tex_width, int expected_tex_height, osg::Camera* master, int cap_img_num)const
{
	osg::GraphicsContext* gc = master->getGraphicsContext();
	unsigned int width = gc->getTraits()->width;//1920
	unsigned int height = gc->getTraits()->height;

	double widthChangeRatio = double(expected_tex_width) / double(width);
	double heigtChangeRatio = double(expected_tex_height) / double(height);
	double aspectRatioChange = widthChangeRatio / heigtChangeRatio;

	osg::Matrixd perspectiveProjMat = master->getProjectionMatrix();

	if (aspectRatioChange != 1.0)
		perspectiveProjMat *= osg::Matrix::scale(1.0 / aspectRatioChange, 1.0, 1.0);

	double fov, aspectRatio, zfar, znear;
	perspectiveProjMat.getPerspective(fov, aspectRatio, znear, zfar);

	// zfar = _terrain_node_bb_info._terrain_boundingbox.radius();//1000;//2*_terrain_node_bb_info._terrain_boundingbox.radius() + _terrain_node_bb_info._terrain_boundingbox.radius();
	std::cout<<"zfar: "<< zfar << std::endl;
	std::cout<<"fov:"<<fov;
	zfar = 0.2f;
	znear = 0.01f;//250;//0.01f;  1896mi
	fov = 60.;// 360.0 / cap_img_num;
	perspectiveProjMat.makePerspective(fov, aspectRatio, znear, zfar);


	//same function as above
	/*osg::Matrixd perspectiveProjMat;
	double fovy = 360.0 / cap_img_num;//���ݴ�ȫ��ģ��Ӱ������������ú��ʵ��ӳ��ǣ�ʹ����Ⱦ�����˴��ν�?
	double aspectRatio = double(expected_tex_width) / double(expected_tex_height);
	double zfar = _terrain_node_bb_info._terrain_boundingbox.radius();
	double znear = 0.01f;
	if (znear >= zfar) zfar += znear;
	perspectiveProjMat.makePerspective(fovy, aspectRatio , znear, zfar);*/ //ʹ�úܽ��Ľ������ֹ��������ã���DO_NOT_COMPUTE_NEAR_FAR���ʹ�÷�ֹZNС��0λ�����֮��?

	return perspectiveProjMat;
}

void ImageCapturer::cofig_rtt_camera(osg::Camera* rtt, osg::Camera* master)const
{
	//�������в�����LOD PLOD�ڵ�ʱ��ABSOLUTE_RF��ABSOLUTE_RF_INHERIT_VIEWPOINTЧ��һ�£�
	//�����������������ͽڵ�ʱ��ABSOLUTE_RF_INHERIT_VIEWPOINT��Ӱ��cullVisitor��pushModelView���̣��޸�_viewPointStack�е����һ���?
	//����traverse(LOD, PLOD)ʱ���ᴥ��cullVisitor����getDistanceToViewPoint������LOD�ڵ�Ŀ��Ӿ���?
	//�����仯���Ӷ����ܵ����޷����ص�Ԥ�ڵ�LOD����
	
	rtt->setRenderOrder(osg::Camera::PRE_RENDER);
	rtt->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	rtt->setClearColor(IRT_BCKCOLOR);
	rtt->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	// rtt->setViewport(0, 0, _cap_cond._capturing_img_size.x(), _cap_cond._capturing_img_size.y());

	osg::Texture2D* cap_img = new osg::Texture2D;
	cap_img->setTextureSize(_cap_cond._capturing_img_size.x(), _cap_cond._capturing_img_size.y());
	
	// RGB
	cap_img->setInternalFormat(GL_RGB8);
	cap_img->setSourceFormat(GL_RGB);
	cap_img->setSourceType(GL_UNSIGNED_BYTE);
	rtt->attach(osg::Camera::COLOR_BUFFER0, cap_img);
	
	// depth
	// cap_img->setInternalFormat(GL_DEPTH_COMPONENT24);
	// cap_img->setSourceFormat(GL_DEPTH_COMPONENT);
	// cap_img->setSourceType(GL_FLOAT);
	// rtt->attach(osg::Camera::DEPTH_BUFFER, cap_img);

	// jiangshan
	// BufferComponent buffer, osg::Texture* texture, unsigned int level, unsigned int face, bool mipMapGeneration,
    // unsigned int multisampleSamples,
    // unsigned int multisampleColorSamples
	// rtt->attach(osg::Camera::COLOR_BUFFER0, cap_img, 0, 0, false, 16, 16);//�����?
	// rtt->attach(osg::Camera::DEPTH_BUFFER, cap_img, 0, 0, false, 16, 16);// no argument
	

	// rtt->attach(osg::Camera::DEPTH_BUFFER, GL_DEPTH_COMPONENT32F);

	rtt->setReferenceFrame(osg::Transform::ABSOLUTE_RF/*ABSOLUTE_RF_INHERIT_VIEWPOINT*/);
	rtt->setProjectionMatrix(createPerspectiveProjection(_cap_cond._capturing_img_size.x(), _cap_cond._capturing_img_size.y(), master, _view_num));

	// rtt->setProjectionMatrix( osg::Matrix::ortho2D(0.0, 1.0, 0.0, 1.0) );
    // rtt->setViewMatrix( osg::Matrix::identity() );

	rtt->setViewport(0, 0, _cap_cond._capturing_img_size.x(), _cap_cond._capturing_img_size.y());

	rtt->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);//scale depth automatic
	//rtt->getOrCreateStateSet()->setMode(GL_LIGHTING , osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);//�رչ���



// double viewDistance = 2.0 * bs.radius();
// double znear = viewDistance - bs.radius();
// double zfar = viewDistance + bs.radius();
// float top = bs.radius();
// float right = bs.radius();
// float bottom = top;
// float left = right;
// //osg左手系坐标(z+向上，x+向右，y+向里)

// //透视投影
// camera->setProjectionMatrixAsFrustum( -left, right, -bottom, top, znear, zfar );
}


// direction
osg::Vec3d ImageCapturer::computeViewDirection(int viewDirection)
{
	//included angle with '(0,1,0)' (1,0,0) (0,-1,0) (-1,0,0) clockwise direction
	// float angle = osg::DegreesToRadians(viewDirection * (360.0 / _view_num));
	if (_image_headig < 0)
	    _image_headig = 360.0 + _image_headig;

	float angle = osg::DegreesToRadians(_image_headig);

	float x = sin(angle), y = cos(angle);

	return osg::Vec3d(x, y, 0);
}

void ImageCapturer::set_capturing_location_attitude(osg::Camera* rtt, int viewDirection)
{
	const osg::Vec3d up(0, 0, 1);
	osg::Vec3d view_direc = computeViewDirection(viewDirection);

	osg::Vec3d observe_center = _camera_eye_pos + view_direc;
	// https://blog.csdn.net/ivan_ljf/article/details/8764737
	// 第一组eyex, eyey,eyez 相机在世界坐标的位置
	// 第二组centerx,centery,centerz 相机镜头对准的物体在世界坐标的位�?
	// 第三组upx,upy,upz 相机向上的方向在世界坐标中的方向
	rtt->setViewMatrixAsLookAt(_camera_eye_pos, observe_center, up);
}

class CapturingPosAttUpdater :public osgGA::CameraManipulator
{
public:
	CapturingPosAttUpdater(osg::Camera* rtt, ImageCapturer* capturer) :_rtt(rtt), _capturer(capturer), _is_first_frame(true), _current_cap_direc(0){}

public:
	virtual void computeHomePosition(const osg::Camera *camera = NULL, bool useBoundingBox = false){}
	virtual void home(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us){}
	virtual void setNode(osg::Node* node){}
	virtual const osg::Node* getNode()const{ return _rtt; }
	virtual osg::Node* getNode(){ return _rtt; }
	virtual void setByMatrix(const osg::Matrixd& viewInverseMatrix){}
	virtual void setByInverseMatrix(const osg::Matrixd& matrix){}
	virtual osg::Matrixd getMatrix()const{ return osg::Matrixd(); }
	virtual osg::Matrixd getInverseMatrix()const{ return osg::Matrixd(); }

public:
	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us)
	{
		if (ea.getEventType() == osgGA::GUIEventAdapter::FRAME)
		{
			if (_is_first_frame){
				_capturer->set_capturing_location_attitude(_rtt, _current_cap_direc);
				_is_first_frame = false;
			}
			else{
				if (g_need_to_change_camera_pos_att){
					_current_cap_direc++;
					g_need_to_change_camera_pos_att = false;
				}
			}	
			_capturer->set_capturing_location_attitude(_rtt, _current_cap_direc);
			return true;
		}
		return false;
	}
private:
	osg::ref_ptr<osg::Camera>		_rtt;
	ImageCapturer*					_capturer;
	bool							_is_first_frame;
	int								_current_cap_direc;
};

class CaptureImagePostCallback :public osg::Camera::DrawCallback
{
public:
	CaptureImagePostCallback(ImageCapturer::ImageContainer& img_container) :_container(img_container), _do_capture(false){}

	void setDoCapturingImage(bool do_cap){ _do_capture = do_cap; }

	virtual void operator () (osg::RenderInfo& renderInfo) const
	{
		if (!_do_capture) return;

		osg::Camera* rrt_cam = renderInfo.getCurrentCamera();
		osg::Camera::BufferAttachmentMap& map = rrt_cam->getBufferAttachmentMap();
		
		// RGB
		const osg::Camera::Attachment& att = map[osg::Camera::COLOR_BUFFER0];
		// DEPTH
		// const osg::Camera::Attachment& att = map[osg::Camera::DEPTH_BUFFER];

		osg::Texture2D* scene_tex = dynamic_cast<osg::Texture2D*> (att._texture.get());

		scene_tex->apply(*renderInfo.getState());

		GLenum pixelformat = scene_tex->getSourceFormat();
		unsigned int width = scene_tex->getTextureWidth();
		unsigned int height = scene_tex->getTextureHeight();

		GLenum dataType = scene_tex->getSourceType();
		osg::ref_ptr<osg::Image> scene_tex_img = new osg::Image;

		int packing = 4;
		scene_tex_img->allocateImage(width, height, 1, pixelformat, dataType, packing);
		glGetTexImage(GL_TEXTURE_2D, 0, pixelformat, dataType, scene_tex_img->data());
		_container.push_back(scene_tex_img);

		_do_capture = false;

		g_need_to_change_camera_pos_att = true;
	}

private:
	ImageCapturer::ImageContainer&		_container;

	mutable bool						_do_capture;

};

void ImageCapturer::simulate()
{
	//int cap_num
	osgViewer::Viewer viewer;
	viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
	osgViewer::Renderer* render = dynamic_cast<osgViewer::Renderer*>(viewer.getCamera()->getRenderer());
	render->setGraphicsThreadDoesCull(false);

	CaptureImagePostCallback* cap_img_callback = new CaptureImagePostCallback(_img_container);
	//CaptureImagePreCallback* cap_img_pre_callback = new CaptureImagePreCallback;
	osg::Camera* rtt = new osg::Camera;
	rtt->setPostDrawCallback(cap_img_callback);//繪制完就保存彩色圖像　		_do_capture = false; g_need_to_change_camera_pos_att = true;
	
	// CapturingPosAttUpdater* updater = new CapturingPosAttUpdater(rtt, this);
	// viewer.setCameraManipulator(updater);//change camera attitude
	// ImageCapturer* capturer
	int current_cap_direc = 0;//first image 0度
	this->set_capturing_location_attitude(rtt, current_cap_direc);

	rtt->addChild(_terrain_node.get());

	PageLODLoadingWatchman* watchman = new PageLODLoadingWatchman;
	osgDB::Registry::instance()->setReadFileCallback(watchman);


	// osg::ref_ptr<osg::Group> root = new osg::Group;
	// root->addChild( rtt );
	// // root->addChild( hudCamera.get() );
	// root->addChild( _terrain_node.get() );
	viewer.setSceneData(rtt);

	viewer.realize();

	cofig_rtt_camera(rtt, viewer.getCamera());

	hideRenderConsole(&viewer);

	if (!g_dp) g_dp = viewer.getDatabasePager();

	//customized rendering
	{
		//�? view point
		// for (unsigned int cap_index = 0; cap_index < cap_num; cap_index++)
		// {
		// 	unsigned int readNodeNumBefore = -1;
		// 	while (!watchman->isFinishLoading(readNodeNumBefore, g_dp)){
		// 		readNodeNumBefore = watchman->getReadNodeNumberTillBeStable();
		// 		viewer.frame();
		// 	}

		// 	cap_img_callback->setDoCapturingImage(true);
		// 	viewer.frame();//rendering one more time to capture the stable scene

		// 	watchman->reset();
		// }

		// only capture one img 
		unsigned int readNodeNumBefore = -1;
		
		while (!watchman->isFinishLoading(readNodeNumBefore, g_dp)){
			// std::cout<<"while "<<std::endl;
			readNodeNumBefore = watchman->getReadNodeNumberTillBeStable();
			// viewer.frame();
			// std::cout<<"readNodeNumBefore "<<readNodeNumBefore<<std::endl;
		}

		cap_img_callback->setDoCapturingImage(true);

		// viewer.run();

		viewer.frame();//rendering one more time to capture the stable scene
		std::cout<<"frame "<<std::endl;

		watchman->reset();
	}
	
	std::cout<<"simulate finish "<<std::endl;
}

bool ImageCapturer::save_capturing_img()const
{
	std::cout<<"save_capturing_img before. "<< std::endl;
	if (_img_container.size() == 0) return false;


	osg::ref_ptr<osg::Image> output_img = new osg::Image;
	osg::Image* flagImg = _img_container[0];
	output_img->allocateImage(flagImg->s() * _img_container.size(), flagImg->t(), 1, flagImg->getPixelFormat(), GL_UNSIGNED_BYTE);

	for (unsigned int i = 0; i < _img_container.size(); i++)
	{
		output_img->copySubImage(flagImg->s() * i, 0, 0, _img_container[i]);
		std::cout<<"for _img_container : "<<i<<std::endl;
	}

	std::cout<<"save name: "<<_cap_cond.gen_output_img_name()<<std::endl;

	return osgDB::writeImageFile(*output_img, _cap_cond.gen_output_img_name());
}

bool ImageCapturer::capture()
{
	// if (irtFileNameUtil::isFileExist(_cap_cond.gen_output_img_name().c_str()))
	// 	return true;
	
	std::cout<<"FileExist "<<std::endl;
	
	// 判断grid四个角点是否在shp 范围�?,如果在范围之内　_camera_eye_pos.set
	// if (!is_camera_pos_within_shpfile()) return true;
	osg::Vec2 pos_lat_lon = _cap_cond._pos_lat_lon;
	float center_height = 0.0;
	getHeightAt(_terrain_node, pos_lat_lon.x(), pos_lat_lon.y(), center_height);
		// 	bool getFlag = getHeightAt(_terrain_node, tile_center.x(), tile_center.y(), center_height);
		// if(getFlag){
		// 	std::cout<<"get height scussced"<<std::endl;
		// }else{
		// 	std::cout<<"get height failed"<<std::endl;
		// }
		
	std::cout<<"getHeight "<<center_height<<std::endl;
	//HeightFieldInfo hfi = computeHeightFieldInfoWithinThisTile();
	//float z_mid = hfi.center();
	//if (center_height < z_mid)
	//center_height = z_mid;
	//center_height =  center_height + hfi.center();
	_camera_eye_pos.set(pos_lat_lon.x(), pos_lat_lon.y(), center_height + _cap_cond._z_lift);
	std::cout<<"camera_eye_pos.set "<<std::endl;

	simulate();//_view_num

	std::cout<<"finish simulate "<<std::endl;

	return save_capturing_img();
}
