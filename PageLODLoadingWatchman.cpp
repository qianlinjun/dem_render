#include "PageLODLoadingWatchman.h"
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/FileUtils>

PageLODLoadingWatchman::PageLODLoadingWatchman():
_readNodeNum(0)
{

}

PageLODLoadingWatchman::~PageLODLoadingWatchman()
{

}

bool PageLODLoadingWatchman::isFinishLoading(unsigned int readNodeNumBefore, osgDB::DatabasePager* DBPager)const
{
	//four condition to ensure PLOD files are loaded fully
	//return readNodeNumBefore == _readNodeNum && DBPager->getFileRequestListSize() == 0 && !DBPager->getDatabaseThread(0)->getActive() && !DBPager->getDatabaseThread(1)->getActive();

	return !DBPager->getRequestsInProgress() && readNodeNumBefore == _readNodeNum;
}

osgDB::ReaderWriter::ReadResult PageLODLoadingWatchman::readNode(const std::string& filename, const osgDB::Options* options)
{
	if (osgDB::containsServerAddress(filename))//HTTP·���ڵ����curl����
	{
		osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension("curl");
		if (rw)
		{
			_readNodeNum++;
			return rw->readNode(filename);
		}
		else
		{
			return  osgDB::ReaderWriter::ReadResult("Warning: Could not find the .curl plugin to read from server.");
		}
	}
	else
	{
		std::string ext = osgDB::getLowerCaseFileExtension(filename);          

		osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension(ext);//����չ������ȡ�ض�ģ�͸�ʽ�Ķ�д��

		//if failed then try to add add ext alias into alias map
		if (!rw)
			if (ext == "laz" || ext == "las")
				osgDB::Registry::instance()->addFileExtensionAlias("laz" , "las");

		//try again
		rw = osgDB::Registry::instance()->getReaderWriterForExtension(ext);//����չ������ȡ�ض�ģ�͸�ʽ�Ķ�д��

		if (!rw)
		{
			return osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED;
		}

		if (!rw->acceptsExtension(ext)) return osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED;

		std::string fileName_ = osgDB::findDataFile( filename, options );
		if (fileName_.empty()) return osgDB::ReaderWriter::ReadResult::FILE_NOT_FOUND;

		osgDB::ReaderWriter::ReadResult res = rw->readNode(fileName_, options);//����Ĭ�ϵĶ�д��������ģ��

		_readNodeNum++;

		return  res;
	}
}
