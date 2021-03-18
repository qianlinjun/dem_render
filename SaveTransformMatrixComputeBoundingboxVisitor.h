/*---------------------------------------------	*\
|����:����任����İ�Χ�м�����				|	
|����:������									|
|����:2019��08��26��							|
|δ��������д���������׷����������				|
/*----------------------------------------------*/

#ifndef SAVE_TRANSFORM_MAT_CBV
#define SAVE_TRANSFORM_MAT_CBV 1

#include <osg/ComputeBoundsVisitor>

// compute bounding box
class SaveTransformMatrixComputeBoundingboxVisitor:public osg::ComputeBoundsVisitor
{
public:
	SaveTransformMatrixComputeBoundingboxVisitor():osg::ComputeBoundsVisitor(){};

	const osg::Matrixd& getLastTransformMatrix()const{return _last_transform_mat;}

public:
	virtual void reset(){osg::ComputeBoundsVisitor::reset();_last_transform_mat.makeIdentity();}

	void apply(osg::Transform& transform)
	{
		osg::Matrix matrix;
		if (!_matrixStack.empty()) matrix = _matrixStack.back();

		transform.computeLocalToWorldMatrix(matrix,this);

		pushMatrix(matrix);

		traverse(transform);

		_last_transform_mat = _matrixStack.back();

		popMatrix();
	}

private:
	osg::Matrixd				_last_transform_mat;
};

#endif
