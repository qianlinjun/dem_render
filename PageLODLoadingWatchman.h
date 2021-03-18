/*----------------------------------------------*\
|版权:武汉大学测绘遥感国家重点实验室openRS团队	|
|功能:确保在当前视点下PLOD节点充分加载			|
|作者:黄翔翔									|
|日期:2019年08月21日							|
|未经允许进行传播，将会追究法律责任				|
/*----------------------------------------------*/

#ifndef IRT_PLOD_LOADING_WATCHMAN
#define IRT_PLOD_LOADING_WATCHMAN 1

#include <osgDB/Callbacks>
#include <osgDB/DatabasePager>

class PageLODLoadingWatchman:public osgDB::ReadFileCallback
{
public:
	PageLODLoadingWatchman();

protected:
	virtual ~PageLODLoadingWatchman();

public:

	unsigned int getReadNodeNumberTillBeStable()const{return _readNodeNum;}

	virtual void reset(){_readNodeNum = 0;}

	//call it at the end of a new frame
	bool isFinishLoading(unsigned int readNodeNumBefore, osgDB::DatabasePager* DBPager)const;

	virtual osgDB::ReaderWriter::ReadResult readNode(const std::string& filename, const osgDB::Options* options);

protected:
	unsigned int						_readNodeNum;
};

#endif