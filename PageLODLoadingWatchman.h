/*----------------------------------------------*\
|��Ȩ:�人��ѧ���ң�й����ص�ʵ����openRS�Ŷ�	|
|����:ȷ���ڵ�ǰ�ӵ���PLOD�ڵ��ּ���			|
|����:������									|
|����:2019��08��21��							|
|δ��������д���������׷����������				|
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