#include "stdio.h"

#include <osg/PolygonMode>
#include <osg/Texture2D>
#include <osg/Group>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>

#include "ImageCapturer.h"

osg::Geode* createScreenQuad( float width, float height, float scale=1.0f )
{
    osg::Geometry* geom = osg::createTexturedQuadGeometry(
        osg::Vec3(), osg::Vec3(width,0.0f,0.0f), osg::Vec3(0.0f,height,0.0f),
        0.0f, 0.0f, width*scale, height*scale );
    osg::ref_ptr<osg::Geode> quad = new osg::Geode;
    quad->addDrawable( geom );
    
    int values = osg::StateAttribute::OFF|osg::StateAttribute::PROTECTED;
    quad->getOrCreateStateSet()->setAttribute( new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL), values );
    quad->getOrCreateStateSet()->setMode( GL_LIGHTING, values );
    return quad.release();
}

osg::Camera* createRTTCamera( osg::Camera::BufferComponent buffer, osg::Texture* tex, bool isAbsolute=false )
{
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setClearColor( osg::Vec4() );
    camera->setClearMask( GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT );
    camera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
    camera->setRenderOrder( osg::Camera::PRE_RENDER );
    if ( tex )
    {
        tex->setFilter( osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR );
        tex->setFilter( osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR );
        camera->setViewport( 0, 0, tex->getTextureWidth(), tex->getTextureHeight() );
        camera->attach( buffer, tex );
    }
    
    if ( isAbsolute )
    {
        camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
        camera->setProjectionMatrix( osg::Matrix::ortho2D(0.0, 1.0, 0.0, 1.0) );
        camera->setViewMatrix( osg::Matrix::identity() );
        camera->addChild( createScreenQuad(1.0f, 1.0f) );
    }
    return camera.release();
}

osg::Camera* createHUDCamera( double left, double right, double bottom, double top )
{
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
    camera->setClearMask( GL_DEPTH_BUFFER_BIT );
    camera->setRenderOrder( osg::Camera::POST_RENDER );
    camera->setAllowEventFocus( false );
    camera->setProjectionMatrix( osg::Matrix::ortho2D(left, right, bottom, top) );
    camera->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    return camera.release();
}




int main( int argc, char **argv )
{
    osg::ArgumentParser arguments( &argc, argv );
    // osg::ref_ptr<osg::Node> scene = osgDB::readNodeFiles( arguments );
    // "cessna.osg"
    // char * model_path = argv[1];
    // if ( !scene ) scene = osgDB::readNodeFile(model_path);

    // dem
    std::string terrain_file_name = argv[1];
    // std::string shp_file_name = argv[2];

    // load terrain data
    osg::Node* terrainNode = osgDB::readNodeFile(terrain_file_name);
    if (!terrainNode) {
		printf("Failed to load terrain and shape files.\n");
		return EXIT_FAILURE;
	}

    // compute boundingbox
    // osg::BoundingBox terrain_bb;
	// osg::ComputeBoundsVisitor cbv;
	// terrainNode->accept(cbv);
	// terrain_bb = cbv.getBoundingBox();//osg::Vec2(terrain_bb.xMin(), terrain_bb.yMin()),  osg::Vec2(terrain_bb.xMax(), terrain_bb.yMax())
    // // 8.48109 46.6005 8.81124 47.0984
    // std::cout<<"terrain_bb xmin ymin xmax ymax"<<terrain_bb.xMin()<< terrain_bb.yMin()<< terrain_bb.xMax()<< terrain_bb.yMax()<<std::endl;

    // get location system
    // osgTerrain::Locator* terrainLocator = getTerrainLocator(terrainNode);
    // region arguments
    // osg::Vec2 ext_min, ext_max;
	// ext_min.x() = atof(argv[3]);
	// ext_min.y() = atof(argv[4]);
	// ext_max.x() = atof(argv[5]);
	// ext_max.y() = atof(argv[6]);
    osg::Vec2 pos_lat_lon;
    pos_lat_lon.x() = atof(argv[2]);
	pos_lat_lon.y() = atof(argv[3]);


	// Desired image dimension.
	osg::Vec2i cap_size;
	cap_size.x() = atoi(argv[4]);
	cap_size.y() = atoi(argv[5]);

    // The z_lift_value of the camera.
	float z_lift = atof(argv[6]);

    std::string output_path = argv[7];

    // The desired image format.
	std::string image_format = argv[8];
    std::cout<<"image_format "<<image_format<<std::endl;
    
    std::string point_name = argv[9];

    //////////////////////////////////////////////////////////////////////////
	// Render images.
    // ImageCapturer::GridTileExtent(ext_min, ext_max)
	ImageCapturer::CapturingCondition condition(point_name, pos_lat_lon, cap_size, z_lift, output_path, image_format);
    std::cout<<"capturecondition "<<std::endl;
	// if (!condition.valid()) {
	// 	printf("CapturingCondition is not valid, please check the setting.\n");
	// 	return EXIT_FAILURE;
	// }


    // ImageCapturer capturer(condition, terrainNode, shpNode, 4);
    ImageCapturer capturer(condition, terrainNode, 4);
    std::cout<<"capturing "<<std::endl;

	if (!capturer.capture()) {
		printf("Error occurs during capturing.\n");
		return EXIT_FAILURE;
	}

    std::cout<<"finish capturer \n"<<std::endl;

	return EXIT_SUCCESS;


    // osg::ref_ptr<osg::Texture2D> tex2D = new osg::Texture2D;
    // tex2D->setTextureSize( 1024, 1024 );
    // tex2D->setInternalFormat( GL_DEPTH_COMPONENT24 );
    // tex2D->setSourceFormat( GL_DEPTH_COMPONENT );
    // tex2D->setSourceType( GL_FLOAT );

    // // create camara buffer 2 texture
    // osg::ref_ptr<osg::Camera> rttCamera = createRTTCamera(osg::Camera::DEPTH_BUFFER, tex2D.get());
    // rttCamera->addChild( scene.get() );

    // osg::ref_ptr<osg::Camera> hudCamera =
    // createHUDCamera(0.0, 1.0, 0.0, 1.0);
    // hudCamera->addChild( createScreenQuad(0.5f, 1.0f) );
    // hudCamera->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex2D.get() );

    // osg::ref_ptr<osg::Group> root = new osg::Group;
    // root->addChild( rttCamera.get() );
    // // root->addChild( hudCamera.get() );
    // root->addChild( scene.get() );

    // osgViewer::Viewer viewer;
    // viewer.getCamera()->setComputeNearFarMode(
    // osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    // viewer.setSceneData( root.get() );
    // return viewer.run();
}













// source code
/* -*-c++-*- OpenSceneGraph Cookbook
 * Chapter 6 Recipe 4
 * Author: Wang Rui <wangray84 at gmail dot com>
*/

// #include <osg/Texture2D>
// #include <osg/Group>
// #include <osgDB/ReadFile>
// #include <osgViewer/Viewer>

// #include "CommonFunctions"

// int main( int argc, char** argv )
// {
//     osg::ArgumentParser arguments( &argc, argv );
//     osg::ref_ptr<osg::Node> scene = osgDB::readNodeFiles( arguments );
//     if ( !scene ) scene = osgDB::readNodeFile("cessna.osg");
    
//     osg::ref_ptr<osg::Texture2D> tex2D = new osg::Texture2D;
//     tex2D->setTextureSize( 1024, 1024 );
//     tex2D->setInternalFormat( GL_DEPTH_COMPONENT24 );
//     tex2D->setSourceFormat( GL_DEPTH_COMPONENT );
//     tex2D->setSourceType( GL_FLOAT );
    
//     osg::ref_ptr<osg::Camera> rttCamera = osgCookBook::createRTTCamera(osg::Camera::DEPTH_BUFFER, tex2D.get());
//     rttCamera->addChild( scene.get() );
    
//     osg::ref_ptr<osg::Camera> hudCamera = osgCookBook::createHUDCamera(0.0, 1.0, 0.0, 1.0);
//     hudCamera->addChild( osgCookBook::createScreenQuad(0.5f, 1.0f) );
//     hudCamera->getOrCreateStateSet()->setTextureAttributeAndModes( 0, tex2D.get() );
    
//     osg::ref_ptr<osg::Group> root = new osg::Group;
//     root->addChild( rttCamera.get() );
//     root->addChild( hudCamera.get() );
//     root->addChild( scene.get() );
    
//     osgViewer::Viewer viewer;
//     viewer.getCamera()->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
//     viewer.setSceneData( root.get() );
//     return viewer.run();
// }

// ./build/GenDem ./data/swizerland-ive/Swiz-17-R4.ive 8.636430 46.841409 1024 1024 0.0 ./output png