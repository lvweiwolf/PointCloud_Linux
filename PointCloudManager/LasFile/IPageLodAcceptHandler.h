//////////////////////////////////////////////////////////////////////
// 文件名称：IPageLodAcceptHandler.h
// 功能描述：PageLod节点遍历处理器接口
// 创建标识：刘庆仙	2022/08/12
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef IPAGELODACCEPTHANDLER_H_
#define IPAGELODACCEPTHANDLER_H_

#include <osg/NodeVisitor>

class CPointCloudpagedLod;

class IPageLodAcceptHandler : public osg::Referenced
{
public:
	/**
	* 处理遍历器接口函数
	* @param [in/out] nv 节点遍历器
	* @return true : 标识已处理 false : 标识未处理
	*/
	virtual bool HandleAccept(osg::NodeVisitor& nv) = 0;
};

#endif // IPAGELODACCEPTHANDLER_H_