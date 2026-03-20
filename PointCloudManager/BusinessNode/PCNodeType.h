/*---------------------------------------------------------------------
*文件名称：EProjectNodeType.h
*功能描述：节点类型枚举类
*创建标识：吴建峰 2026/1/24
----------------------------------------------------------------------*/
#ifndef PC_NODE_TYPE_H_
#define PC_NODE_TYPE_H_

enum EProjectNodeType
{
	eProjectBegin = 1000, // 工程开始

	/*---------根节点--------------*/
	eBnsProjectRoot = 1001, // 工程根节点
	eBnsProjectInfo = 1002, // 工程信息

	eBnsPointCloudLodNodeArr = 2000,
	eBnsPointCloudLodNode = 2001,
};

#endif // PC_NODE_TYPE_H_
